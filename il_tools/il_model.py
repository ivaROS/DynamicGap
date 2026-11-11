"""
Gap scorer: learns which gap the operator picks from DynamicGap's candidates.

Every feasible candidate of a tick gets a score; the robot follows the highest
one and switches when another gap overtakes it. Training maximises the
probability (softmax over the tick's candidates) of the operator's gap.

    s_i = -(w_terminal * dg_terminal_cost_i + w_clearance * dg_clearance_cost_i) + delta_i

The first term is DynamicGap's own rule with learned weights (the linear model);
delta_i is the network's correction. With network=False this is the linear
model; the network's last layer starts at zero, so training starts from it.
DynamicGap's costs are recorded in sessions of schema 2 or later; before, the
terminal cost is rebuilt from the trajectory end and the local waypoint
(dg_terms) and the clearance cost is left out.

Inputs are the raw quantities of an example (il_dataset.py), robot frame:
the network learns its own features. Input groups (INPUT_GROUPS) can be left out
or added back for ablations; the inputs computed stay the same, the model picks
its columns. Normalisation is stored in the model so an exported model takes raw
inputs.

The default model (il_train.DEFAULTS) has no term on DynamicGap's costs and reads
the scene: every edge of the scan (both points of every raw gap, schema 4) with
its velocity over the ground, linked to its neighbours. The links are the scene's
tokens: a gap link between the two points of a raw gap, an obstacle link between
the two ends of the obstacle in between. Each candidate reads them through
attention (SceneAttention), then the candidates see each other (CandidateAttention).
Per candidate: its two points and their ground velocities; per tick: the robot's
own velocity, the goal and the local waypoint.
"""

import math

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import il_bags


TERMINAL_COST_CAP = 20.0   # m (times Q_f)
Q_F = 0.05                 # DynamicGap's Q_f (traj.Q_f) in every recorded session
CLEARANCE_COST_CAP = 10.0  # inf (pose inside the inflated robot) becomes this
MASKED_SCORE = -1e9        # score of a padded or infeasible candidate

CANDIDATE_FEATURES = (
    ["left_x", "left_y", "right_x", "right_y", "width", "bearing_cos", "bearing_sin"]
    + ["left_model_" + k for k in ("x", "y", "vx", "vy")]
    + ["right_model_" + k for k in ("x", "y", "vx", "vy")]
    + ["terminal_x", "terminal_y"]
    # new features go last: a model trained on a shorter list still runs
    + ["cost_infinite"]
    # velocity over the ground of the scene edges the gap points are (schema 4)
    + ["left_gvx", "left_gvy", "right_gvx", "right_gvy", "edge_vel_known"])
CONTEXT_FEATURES = ["vel_x", "vel_y", "vel_yaw", "goal_x", "goal_y", "waypoint_x", "waypoint_y"]
# one scene link: obstacle (1) or gap (0), its two edges' positions and ground velocities
SCENE_FEATURES = ["is_obstacle", "a_x", "a_y", "a_gvx", "a_gvy", "b_x", "b_y", "b_gvx", "b_gvy"]
EDGE_MATCH_RAYS = 3  # a candidate's point and a scene edge at most this many scan rays apart are the same

INPUT_GROUPS = {
    # the candidate gap's own two points (not the scene edges)
    "edges": ["left_x", "left_y", "right_x", "right_y", "width", "bearing_cos", "bearing_sin"],
    "ped_pos": ["left_model_x", "left_model_y", "right_model_x", "right_model_y"],
    "ped_vel": ["left_model_vx", "left_model_vy", "right_model_vx", "right_model_vy"],
    "terminal": ["terminal_x", "terminal_y"],   # trajectory end point; off by default
    "robot_vel": ["vel_x", "vel_y", "vel_yaw"],
    "goal": ["goal_x", "goal_y"],
    "waypoint": ["waypoint_x", "waypoint_y"],
    # 1 where DynamicGap's cost is infinite (a pose inside the inflated robot, or a
    # path under 2 poses): the capped cost alone does not tell it from a large one
    "cost_infinite": ["cost_infinite"],
    # ground velocities of the candidate's points; ped_vel are DynamicGap's planning velocities
    "gvel": ["left_gvx", "left_gvy", "right_gvx", "right_gvy", "edge_vel_known"],
}
DEFAULT_GROUPS = ["edges", "gvel", "robot_vel", "goal", "waypoint"]


def inputs_of(groups):
    return [f for g in groups for f in INPUT_GROUPS[g]]


def finite(x, default=0.0):
    return x if x is not None and math.isfinite(x) else default


def ground_velocity(edge, robot_vel):
    """Velocity over the ground of a scene edge, robot frame axes: the estimator's
    velocity relative to the robot plus the robot's own (Estimator::isolateGapDynamics).
    A still wall is 0 whether the robot moves or not. None without an estimate."""
    vx, vy = edge["model_vel"]
    if not (math.isfinite(vx) and math.isfinite(vy)):
        return None
    return [vx + robot_vel[0], vy + robot_vel[1]]


def edge_position(edge):
    return [edge["range"] * math.cos(edge["theta"]), edge["range"] * math.sin(edge["theta"])]


def edge_at(edges, scan_idx):
    """Position in edges of the scene edge at scan ray scan_idx (nearest, at most
    EDGE_MATCH_RAYS away); None if none or scan_idx is unknown."""
    if scan_idx is None or scan_idx < 0 or not edges:
        return None
    best = min(range(len(edges)), key=lambda i: abs(edges[i]["scan_idx"] - scan_idx))
    return best if abs(edges[best]["scan_idx"] - scan_idx) <= EDGE_MATCH_RAYS else None


def scene_links(edges, robot_vel):
    """The scene's tokens (SCENE_FEATURES): one per link. The edges are in scan order,
    two per raw gap (right point, then left point): a gap link from each right point
    to its left point, an obstacle link from each left point to the next edge."""
    def state(i):
        gv = ground_velocity(edges[i], robot_vel)
        return edge_position(edges[i]) + (gv if gv is not None else [0.0, 0.0])

    links = []
    for i, e in enumerate(edges):
        if i % 2 == 0 and 0 <= e["gap_neighbor"] < len(edges):
            links.append([0.0] + state(i) + state(e["gap_neighbor"]))
        elif i % 2 == 1 and 0 <= e["obstacle_neighbor"] < len(edges):
            links.append([1.0] + state(i) + state(e["obstacle_neighbor"]))
    return [[finite(x) for x in link] for link in links]


def candidate_vector(c, edges=(), robot_vel=(0.0, 0.0, 0.0)):
    left = [c["left_range"] * math.cos(c["left_theta"]), c["left_range"] * math.sin(c["left_theta"])]
    right = [c["right_range"] * math.cos(c["right_theta"]), c["right_range"] * math.sin(c["right_theta"])]
    gv = [None, None]
    for k, side in enumerate(("left", "right")):
        i = edge_at(edges, c.get(side + "_scan_idx"))
        if i is not None:
            gv[k] = ground_velocity(edges[i], robot_vel)
    known = gv[0] is not None and gv[1] is not None
    v = (left + right + [math.hypot(left[0] - right[0], left[1] - right[1]),
                         math.cos(c["bearing"]), math.sin(c["bearing"])]
         + c["left_model"] + c["right_model"]
         + c.get("terminal_rbt", [0.0, 0.0])
         + [1.0 if c.get("dg_cost") is not None and math.isinf(c["dg_cost"]) else 0.0]
         + (gv[0] or [0.0, 0.0]) + (gv[1] or [0.0, 0.0]) + [1.0 if known else 0.0])
    return [finite(x) for x in v]


def rebuilt_terminal_cost(c, waypoint):
    """DynamicGap's terminal cost, Q_f * |trajectory end - local waypoint| (both robot
    frame, TrajectoryEvaluator::terminalGoalCost), from what every schema records;
    None if the trajectory end is unknown."""
    end = c.get("terminal_rbt")
    if end is None or waypoint is None or not all(math.isfinite(x) for x in list(end) + list(waypoint)):
        return None
    return Q_F * math.hypot(end[0] - waypoint[0], end[1] - waypoint[1])


def dg_terms(c, waypoint=None):
    """DynamicGap's two cost terms (capped), whether they are available, and whether
    the terminal cost was rebuilt. Sessions before schema 2 did not record the costs:
    the terminal cost is rebuilt from the trajectory end and the local waypoint, the
    clearance cost (needs the propagated scans) is set to 0. It is 0 for every
    candidate of such a tick, so it does not change that tick's choice."""
    t, k = c.get("dg_terminal_cost"), c.get("dg_clearance_cost")
    if t is not None and k is not None and not math.isnan(t) and not math.isnan(k):
        return [min(t, TERMINAL_COST_CAP), min(k, CLEARANCE_COST_CAP)], 1.0, False
    t = rebuilt_terminal_cost(c, waypoint)
    if t is None:
        return [0.0, 0.0], 0.0, False
    return [min(t, TERMINAL_COST_CAP), 0.0], 1.0, True


def selectable(c):
    """The candidates the model may choose: those the operator could choose."""
    return bool(c["feasible"]) and c.get("shown", True)


def options_only(ex):
    """The example with only the operator's options as candidates; label and dg_pick
    follow. The operator's gap stays (shown is missing before schema 2)."""
    keep = [i for i, c in enumerate(ex["candidates"]) if selectable(c) or i == ex["label"]]
    if len(keep) == len(ex["candidates"]):
        return ex
    dg_pick = ex.get("dg_pick")
    return dict(ex, candidates=[ex["candidates"][i] for i in keep], label=keep.index(ex["label"]),
                dg_pick=dg_pick if dg_pick is None or dg_pick < 0
                else keep.index(dg_pick) if dg_pick in keep else -1)


def featurize(ex):
    """One example of il_dataset.py as arrays; None if no candidate is selectable."""
    cands = ex["candidates"]
    mask = np.array([selectable(c) for c in cands])
    # the operator chose it, so it was selectable (shown is missing before schema 2)
    mask[ex["label"]] = True
    dg = [dg_terms(c, ex.get("local_waypoint")) for c in cands]
    edges = ex.get("edges", [])
    return {
        "cand": np.array([candidate_vector(c, edges, ex["robot_vel"]) for c in cands], dtype=np.float32),
        "scene": np.array(scene_links(edges, ex["robot_vel"]), dtype=np.float32).reshape(-1, len(SCENE_FEATURES)),
        "dg": np.array([d[0] for d in dg], dtype=np.float32),
        "dg_avail": np.array([d[1] for d in dg], dtype=np.float32),
        # the clearance cost was recorded (not only a rebuilt terminal cost)
        "dg_clearance": any(d[1] and not d[2] for d in dg),
        "ctx": np.array([finite(x) for x in ex["robot_vel"] + ex["goal"] + ex["local_waypoint"]], dtype=np.float32),
        "mask": mask,
        "label": ex["label"],
    }


def gap_identity(c):
    """None if unknown: no model ids recorded (older datasets) or a gap point without
    a model (id -1), which the planner cannot tell from another gap."""
    ids = (c.get("left_model_id"), c.get("right_model_id"))
    return None if None in ids else il_bags.known_identity(*ids)


def decision_weights(examples, switch_weight=1.0, disagree_weight=1.0):
    """Weight per example: 1, plus switch_weight where the operator's gap differs
    from their gap on the previous example of the episode, plus disagree_weight
    where DynamicGap would have picked another gap. Consecutive ticks are near
    duplicates; these ticks carry the decisions. Needs model ids and dg_pick,
    so older datasets get weight 1 throughout."""
    weights, prev = [], {}
    for ex in examples:
        w = 1.0
        key = (ex["session"], ex["episode"])
        ident = gap_identity(ex["candidates"][ex["label"]])
        if ident is not None:
            # against the last known gap of the episode
            if prev.get(key) is not None and prev[key] != ident:
                w += switch_weight
            prev[key] = ident
        if ex.get("dg_pick") is not None and ex["dg_pick"] >= 0 and ex["dg_pick"] != ex["label"]:
            w += disagree_weight
        weights.append(w)
    return weights


def collate(items, weights=None):
    """Pad a list of featurize() outputs to the largest candidate count."""
    b, n = len(items), max(len(it["mask"]) for it in items)
    # at least one (padded) scene token, so the attention has keys
    s = max([1] + [len(it.get("scene", ())) for it in items])
    cand = torch.zeros(b, n, len(CANDIDATE_FEATURES))
    dg = torch.zeros(b, n, 2)
    dg_avail = torch.zeros(b, n)
    mask = torch.zeros(b, n, dtype=torch.bool)
    scene = torch.zeros(b, s, len(SCENE_FEATURES))
    scene_mask = torch.zeros(b, s, dtype=torch.bool)
    for i, it in enumerate(items):
        m = len(it["mask"])
        cand[i, :m] = torch.from_numpy(it["cand"])
        dg[i, :m] = torch.from_numpy(it["dg"])
        dg_avail[i, :m] = torch.from_numpy(it["dg_avail"])
        mask[i, :m] = torch.from_numpy(it["mask"])
        k = len(it.get("scene", ()))
        if k:
            scene[i, :k] = torch.from_numpy(it["scene"])
            scene_mask[i, :k] = True
    return {
        "cand": cand, "dg": dg, "dg_avail": dg_avail, "mask": mask, "scene": scene, "scene_mask": scene_mask,
        "ctx": torch.from_numpy(np.stack([it["ctx"] for it in items])),
        "label": torch.tensor([it["label"] for it in items]),
        "weight": torch.ones(b) if weights is None else torch.tensor(weights, dtype=torch.float32),
    }


def normalizer(items):
    """Mean and std of the candidate (selectable ones), context and scene inputs."""
    cand = np.concatenate([it["cand"][it["mask"]] for it in items])
    ctx = np.stack([it["ctx"] for it in items])
    scene = np.concatenate([it["scene"] for it in items if len(it.get("scene", ()))]
                           or [np.zeros((1, len(SCENE_FEATURES)), dtype=np.float32)])
    # constant inputs (std 0) are left unscaled
    std = lambda x: np.where(x.std(0) > 1e-6, x.std(0), 1.0)  # noqa: E731
    return {"cand_mean": cand.mean(0), "cand_std": std(cand), "ctx_mean": ctx.mean(0), "ctx_std": std(ctx),
            "scene_mean": scene.mean(0), "scene_std": std(scene)}


NORM_KEYS = ("cand_mean", "cand_std", "ctx_mean", "ctx_std", "scene_mean", "scene_std")


def norm_of(state):
    """The normalisation buffers of a state dict (models from before the scene have no scene ones)."""
    return {k: state[k] for k in NORM_KEYS if k in state}


def mlp(sizes, dropout=0.0):
    layers = []
    for a, b in zip(sizes[:-1], sizes[1:]):
        layers += [nn.Linear(a, b), nn.ReLU(), nn.Dropout(dropout)]
    return nn.Sequential(*layers[:-2])  # no activation after the last layer


class CandidateAttention(nn.Module):
    """One pre-norm self-attention block over the candidates of a tick, so a gap's
    score can depend on the others. No positional encoding: candidate order does
    not matter. Written out (not nn.MultiheadAttention) to export to ONNX."""

    def __init__(self, dim, heads=4, dropout=0.0):
        super().__init__()
        assert dim % heads == 0
        self.heads, self.head_dim = heads, dim // heads
        self.norm1, self.norm2 = nn.LayerNorm(dim), nn.LayerNorm(dim)
        self.qkv = nn.Linear(dim, 3 * dim)
        self.out = nn.Linear(dim, dim)
        self.ff = mlp([dim, 2 * dim, dim], dropout)
        self.drop = nn.Dropout(dropout)

    def forward(self, h, mask):
        b, n = h.shape[0], h.shape[1]
        q, k, v = self.qkv(self.norm1(h)).view(b, n, 3, self.heads, self.head_dim).permute(2, 0, 3, 1, 4).unbind(0)
        att = (q @ k.transpose(-1, -2)) / math.sqrt(self.head_dim)
        att = att.masked_fill(~mask[:, None, None, :], MASKED_SCORE).softmax(-1)
        h = h + self.drop(self.out((att @ v).transpose(1, 2).reshape(b, n, self.heads * self.head_dim)))
        return h + self.drop(self.ff(self.norm2(h)))


class SceneAttention(nn.Module):
    """One pre-norm cross-attention block: each candidate reads the scene's links,
    then a feed-forward. A tick without scene links leaves the candidates unchanged."""

    def __init__(self, dim, heads=4, dropout=0.0):
        super().__init__()
        assert dim % heads == 0
        self.heads, self.head_dim = heads, dim // heads
        self.norm_q, self.norm_kv, self.norm2 = nn.LayerNorm(dim), nn.LayerNorm(dim), nn.LayerNorm(dim)
        self.q = nn.Linear(dim, dim)
        self.kv = nn.Linear(dim, 2 * dim)
        self.out = nn.Linear(dim, dim)
        self.ff = mlp([dim, 2 * dim, dim], dropout)
        self.drop = nn.Dropout(dropout)

    def forward(self, h, s, scene_mask):
        b, n, m = h.shape[0], h.shape[1], s.shape[1]
        q = self.q(self.norm_q(h)).view(b, n, self.heads, self.head_dim).transpose(1, 2)
        k, v = self.kv(self.norm_kv(s)).view(b, m, 2, self.heads, self.head_dim).permute(2, 0, 3, 1, 4).unbind(0)
        att = (q @ k.transpose(-1, -2)) / math.sqrt(self.head_dim)
        att = att.masked_fill(~scene_mask[:, None, None, :], MASKED_SCORE).softmax(-1)
        read = self.out((att @ v).transpose(1, 2).reshape(b, n, self.heads * self.head_dim))
        read = read * scene_mask.any(-1).float()[:, None, None]
        h = h + self.drop(read)
        return h + self.drop(self.ff(self.norm2(h)))


class GapScorer(nn.Module):
    """network=False: the linear model (DynamicGap's terms, learned weights).
    residual=False: the network alone. attention=False: each gap scored on its own.
    scene=True: the candidates read the scene's links (needs scene_mean/std in norm).
    inputs: the feature names the network uses (default inputs_of(DEFAULT_GROUPS))."""

    def __init__(self, norm, hidden=64, network=True, residual=True, attention=True, heads=4, dropout=0.1,
                 inputs=None, scene=False):
        super().__init__()
        assert network or residual
        self.network, self.residual, self.attention = network, residual, attention
        self.scene = scene and network
        self.inputs = list(inputs_of(DEFAULT_GROUPS) if inputs is None else inputs)
        for k, v in norm.items():
            self.register_buffer(k, torch.as_tensor(v, dtype=torch.float32))
        cand_idx = [CANDIDATE_FEATURES.index(f) for f in self.inputs if f in CANDIDATE_FEATURES]
        ctx_idx = [CONTEXT_FEATURES.index(f) for f in self.inputs if f in CONTEXT_FEATURES]
        assert len(cand_idx) + len(ctx_idx) == len(self.inputs), "unknown input"
        self.register_buffer("cand_idx", torch.tensor(cand_idx, dtype=torch.long))
        self.register_buffer("ctx_idx", torch.tensor(ctx_idx, dtype=torch.long))
        # DynamicGap's rule: -(terminal + clearance)
        self.dg_weight = nn.Parameter(torch.ones(2)) if residual else None
        if network:
            self.cand_enc = mlp([len(cand_idx), hidden, hidden], dropout)
            self.ctx_enc = mlp([len(ctx_idx), hidden, hidden], dropout)
            if self.scene:
                self.scene_enc = mlp([len(SCENE_FEATURES), hidden, hidden], dropout)
                self.read_scene = SceneAttention(hidden, heads, dropout)
            self.mix = CandidateAttention(hidden, heads, dropout) if attention else None
            self.head = mlp([2 * hidden, hidden, 1], dropout)
            # start with no correction: the model starts as the linear model
            nn.init.zeros_(self.head[-1].weight)
            nn.init.zeros_(self.head[-1].bias)

    def forward(self, cand, ctx, dg, dg_avail, mask, scene=None, scene_mask=None):
        """cand [B,N,F], ctx [B,C], dg [B,N,2], dg_avail [B,N], mask [B,N] (bool),
        scene [B,M,S] and scene_mask [B,M] (bool; used with scene=True).
        Returns scores [B,N]; candidates outside the mask get MASKED_SCORE."""
        score = torch.zeros(mask.shape, device=mask.device)
        if self.residual:
            score = score - (dg * self.dg_weight).sum(-1) * dg_avail
        if self.network:
            # select, then normalise: a model trained on fewer features has shorter statistics
            e = self.cand_enc((cand.index_select(-1, self.cand_idx) - self.cand_mean[self.cand_idx])
                              / self.cand_std[self.cand_idx])
            c = self.ctx_enc((ctx.index_select(-1, self.ctx_idx) - self.ctx_mean[self.ctx_idx])
                             / self.ctx_std[self.ctx_idx])
            if self.scene or self.mix is not None:
                # context goes in with every candidate
                e = e + c[:, None, :]
            if self.scene:
                # each candidate reads the scene
                s = self.scene_enc((scene - self.scene_mean) / self.scene_std)
                e = self.read_scene(e, s, scene_mask)
            if self.mix is not None:
                # then the candidates see each other
                e = self.mix(e, mask)
            h = torch.cat([e, c[:, None, :].expand_as(e)], -1)
            score = score + self.head(h).squeeze(-1)
        return score.masked_fill(~mask, MASKED_SCORE)


def scores(model, batch):
    return model(batch["cand"], batch["ctx"], batch["dg"], batch["dg_avail"], batch["mask"],
                 batch.get("scene"), batch.get("scene_mask"))


def loss(model, batch):
    """Weighted cross-entropy of the operator's gap over the tick's candidates."""
    nll = F.cross_entropy(scores(model, batch), batch["label"], reduction="none")
    return (nll * batch["weight"]).sum() / batch["weight"].sum()


def top1(model, batch):
    """Share of examples where the highest-scoring gap is the operator's."""
    return (scores(model, batch).argmax(-1) == batch["label"]).float().mean().item()

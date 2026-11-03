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
the network learns its own features. By default (DEFAULT_INPUTS) per gap only
its edges and the pedestrian estimates at them; the gap goal, the trajectory
and the gap tube are DynamicGap's own constructions and reach the model only
through its costs. Input groups (INPUT_GROUPS) can be left out or added back
for ablations; the inputs computed stay the same, the model picks its columns. Normalisation is stored in the model so
an exported model takes raw inputs.
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
    + ["cost_infinite"])
CONTEXT_FEATURES = ["vel_x", "vel_y", "vel_yaw", "goal_x", "goal_y", "waypoint_x", "waypoint_y"]

INPUT_GROUPS = {
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
}
DEFAULT_GROUPS = ["edges", "ped_pos", "ped_vel", "robot_vel", "goal", "waypoint", "cost_infinite"]


def inputs_of(groups):
    return [f for g in groups for f in INPUT_GROUPS[g]]


def finite(x, default=0.0):
    return x if x is not None and math.isfinite(x) else default


def candidate_vector(c):
    left = [c["left_range"] * math.cos(c["left_theta"]), c["left_range"] * math.sin(c["left_theta"])]
    right = [c["right_range"] * math.cos(c["right_theta"]), c["right_range"] * math.sin(c["right_theta"])]
    v = (left + right + [math.hypot(left[0] - right[0], left[1] - right[1]),
                         math.cos(c["bearing"]), math.sin(c["bearing"])]
         + c["left_model"] + c["right_model"]
         + c.get("terminal_rbt", [0.0, 0.0])
         + [1.0 if c.get("dg_cost") is not None and math.isinf(c["dg_cost"]) else 0.0])
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
    return {
        "cand": np.array([candidate_vector(c) for c in cands], dtype=np.float32),
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
    cand = torch.zeros(b, n, len(CANDIDATE_FEATURES))
    dg = torch.zeros(b, n, 2)
    dg_avail = torch.zeros(b, n)
    mask = torch.zeros(b, n, dtype=torch.bool)
    for i, it in enumerate(items):
        m = len(it["mask"])
        cand[i, :m] = torch.from_numpy(it["cand"])
        dg[i, :m] = torch.from_numpy(it["dg"])
        dg_avail[i, :m] = torch.from_numpy(it["dg_avail"])
        mask[i, :m] = torch.from_numpy(it["mask"])
    return {
        "cand": cand, "dg": dg, "dg_avail": dg_avail, "mask": mask,
        "ctx": torch.from_numpy(np.stack([it["ctx"] for it in items])),
        "label": torch.tensor([it["label"] for it in items]),
        "weight": torch.ones(b) if weights is None else torch.tensor(weights, dtype=torch.float32),
    }


def normalizer(items):
    """Mean and std of the candidate (selectable ones) and context inputs."""
    cand = np.concatenate([it["cand"][it["mask"]] for it in items])
    ctx = np.stack([it["ctx"] for it in items])
    # constant inputs (std 0) are left unscaled
    std = lambda x: np.where(x.std(0) > 1e-6, x.std(0), 1.0)  # noqa: E731
    return {"cand_mean": cand.mean(0), "cand_std": std(cand), "ctx_mean": ctx.mean(0), "ctx_std": std(ctx)}


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


class GapScorer(nn.Module):
    """network=False: the linear model (DynamicGap's terms, learned weights).
    residual=False: the network alone. attention=False: each gap scored on its own.
    inputs: the feature names the network uses (default inputs_of(DEFAULT_GROUPS))."""

    def __init__(self, norm, hidden=64, network=True, residual=True, attention=True, heads=4, dropout=0.1,
                 inputs=None):
        super().__init__()
        assert network or residual
        self.network, self.residual, self.attention = network, residual, attention
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
            self.mix = CandidateAttention(hidden, heads, dropout) if attention else None
            self.head = mlp([2 * hidden, hidden, 1], dropout)
            # start with no correction: the model starts as the linear model
            nn.init.zeros_(self.head[-1].weight)
            nn.init.zeros_(self.head[-1].bias)

    def forward(self, cand, ctx, dg, dg_avail, mask):
        """cand [B,N,F], ctx [B,C], dg [B,N,2], dg_avail [B,N], mask [B,N] (bool).
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
            if self.mix is not None:
                # context goes in with every candidate, then the candidates see each other
                e = self.mix(e + c[:, None, :], mask)
            h = torch.cat([e, c[:, None, :].expand_as(e)], -1)
            score = score + self.head(h).squeeze(-1)
        return score.masked_fill(~mask, MASKED_SCORE)


def scores(model, batch):
    return model(batch["cand"], batch["ctx"], batch["dg"], batch["dg_avail"], batch["mask"])


def loss(model, batch):
    """Weighted cross-entropy of the operator's gap over the tick's candidates."""
    nll = F.cross_entropy(scores(model, batch), batch["label"], reduction="none")
    return (nll * batch["weight"]).sum() / batch["weight"].sum()


def top1(model, batch):
    """Share of examples where the highest-scoring gap is the operator's."""
    return (scores(model, batch).argmax(-1) == batch["label"]).float().mean().item()

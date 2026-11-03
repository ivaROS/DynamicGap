"""
Training and evaluation of the gap scorer (il_model.py) on a dataset of
il_dataset.py ({train,val,test}.jsonl.gz).

Early stopping on the validation loss. Besides agreement with the operator,
evaluation measures how sticky the choices are: the share of consecutive
examples where the chosen gap changes, and one-example blips (A, B, A).
"""

import csv
import gzip
import json
import math
import os
import random
import sys
import time

import numpy as np
import torch

import il_model


DEFAULTS = {
    # no term on DynamicGap's costs; the network reads the scene's edges instead
    "network": True, "residual": False, "attention": True, "scene": True,
    "groups": il_model.DEFAULT_GROUPS,
    "hidden": 64, "dropout": 0.1,
    "epochs": 300, "batch": 128, "lr": 1e-3, "weight_decay": 1e-4, "patience": 50,
    "switch_weight": 1.0, "disagree_weight": 1.0,
    # weight on DynamicGap's terminal cost (Q_f * distance to the local waypoint):
    # starting value, and a floor kept through training (the pull toward the waypoint)
    "terminal_weight_init": 1.0, "terminal_weight_min": None,
    "seed": 0,
}


def load_split(data_dir, split):
    path = os.path.join(data_dir, split + ".jsonl.gz")
    if not os.path.exists(path):
        return []
    with gzip.open(path, "rt") as f:
        return [il_model.options_only(json.loads(line)) for line in f]


def featurize_splits(tr, va, te):
    return {k: [il_model.featurize(ex) for ex in v] for k, v in (("train", tr), ("val", va), ("test", te))}


def has_dg_costs(items):
    """Whether the linear term can train: some candidate has DynamicGap's terminal cost (recorded or rebuilt)."""
    return any(it["dg_avail"].any() for it in items)


def has_clearance_costs(items):
    return any(it["dg_clearance"] for it in items)


def has_scene(items):
    """Whether the dataset records the scene's edges (schema 4)."""
    return any(len(it["scene"]) for it in items)


def seed_all(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def sequences(examples):
    """Example positions per episode, in tick order."""
    by_episode = {}
    for i, ex in enumerate(examples):
        by_episode.setdefault((ex["session"], ex["episode"]), []).append(i)
    return [sorted(idx, key=lambda i: examples[i]["obs_tick"]) for idx in by_episode.values()]


def stickiness(examples, picks):
    """Switch rate and blip rate of a sequence of picks (candidate positions); None
    without model ids (older datasets). A gap without identity is a gap of its own,
    as the planner treats it."""
    same = lambda a, b: a is not None and a == b  # noqa: E731
    # whether the dataset has model ids at all (stops at the first known one)
    if not any(il_model.gap_identity(c) is not None for ex in examples for c in ex["candidates"]):
        return {"switch_rate": None, "blip_rate": None}
    switches = blips = steps = 0
    for seq in sequences(examples):
        ids = [il_model.gap_identity(examples[i]["candidates"][picks[i]]) for i in seq]
        steps += max(len(ids) - 1, 0)
        switches += sum(not same(a, b) for a, b in zip(ids, ids[1:]))
        blips += sum(same(a, c) and not same(a, b) for a, b, c in zip(ids, ids[1:], ids[2:]))
    return {"switch_rate": switches / max(steps, 1), "blip_rate": blips / max(steps, 1)}


def evaluate(model, examples, items):
    """Loss and agreement with the operator; stickiness of the model and of the operator."""
    if not examples:
        return {}
    model.eval()
    batch = il_model.collate(items)
    with torch.no_grad():
        s = il_model.scores(model, batch)
        nll = torch.nn.functional.cross_entropy(s, batch["label"]).item()
    picks = s.argmax(-1).tolist()
    labels = [ex["label"] for ex in examples]
    switched = [w > 1 for w in il_model.decision_weights(examples, 1.0, 0.0)]
    hit = [p == l for p, l in zip(picks, labels)]
    m = {
        "n": len(examples), "nll": nll, "top1": float(np.mean(hit)),
        "top1_at_switches": float(np.mean([h for h, sw in zip(hit, switched) if sw])) if any(switched) else None,
        "chance": float((1.0 / batch["mask"].sum(1).float()).mean()),
    }
    dg = [ex.get("dg_pick") for ex in examples]
    if all(d is not None and d >= 0 for d in dg):
        m["dg_top1"] = float(np.mean([d == l for d, l in zip(dg, labels)]))
    model_st, op_st = stickiness(examples, picks), stickiness(examples, labels)
    m.update({"model_" + k: v for k, v in model_st.items()})
    m.update({"operator_" + k: v for k, v in op_st.items()})
    return m


def build_model(cfg, norm):
    return il_model.GapScorer(norm, hidden=cfg["hidden"], network=cfg["network"], residual=cfg["residual"],
                              attention=cfg["attention"], dropout=cfg["dropout"],
                              inputs=il_model.inputs_of(cfg["groups"]), scene=cfg["scene"])


def fmt(m, keys=("nll", "top1", "top1_at_switches", "model_switch_rate")):
    return " ".join("{} {}".format(k, "-" if m.get(k) is None else "%.3f" % m[k]) for k in keys)


def train(data_dir, cfg=None, out_dir=None, log=sys.stdout, log_every=1, data=None, items=None):
    """Train one model; returns (model, result). data: the three splits already
    loaded, items: featurized (featurize_splits) (for ablations, so each run does
    not redo them)."""
    cfg = dict(DEFAULTS, **(cfg or {}))
    seed_all(cfg["seed"])
    tr, va, te = data or [load_split(data_dir, s) for s in ("train", "val", "test")]
    # the validation split picks the epoch that is kept
    for split, examples in (("train", tr), ("val", va)):
        if not examples:
            raise ValueError("the {} split of {} is missing or empty".format(split, data_dir))
    items = items or featurize_splits(tr, va, te)
    if cfg["residual"] and not has_dg_costs(items["train"]):
        # the term would never train, yet the planner always has the costs and would apply it
        if not cfg["network"]:
            raise ValueError("the linear model needs DynamicGap's costs")
        print("note: no DynamicGap costs in this dataset, training without the linear term", file=log)
        cfg["residual"] = False
    if cfg["scene"] and cfg["network"] and not has_scene(items["train"]):
        print("note: no scene edges in this dataset (recorded before schema 4), training without the scene",
              file=log)
        cfg["scene"] = False
    weights = il_model.decision_weights(tr, cfg["switch_weight"], cfg["disagree_weight"])
    model = build_model(cfg, il_model.normalizer(items["train"]))
    if cfg["residual"]:
        with torch.no_grad():
            floor = cfg["terminal_weight_min"]
            model.dg_weight[0] = max(cfg["terminal_weight_init"], -math.inf if floor is None else floor)
    if cfg["residual"] and not has_clearance_costs(items["train"]):
        # only rebuilt terminal costs (schema < 2): the clearance weight would never
        # train, and the planner applies it; leave DynamicGap's clearance cost out
        print("note: no recorded clearance costs, its weight is fixed at 0", file=log)
        with torch.no_grad():
            model.dg_weight[1] = 0.0
    opt = torch.optim.Adam(model.parameters(), lr=cfg["lr"], weight_decay=cfg["weight_decay"])

    history, best, best_state, since_best = [], None, None, 0
    t0 = time.time()
    print("train {} val {} test {} examples, {} parameters".format(
        len(tr), len(va), len(te), sum(p.numel() for p in model.parameters())), file=log)
    for epoch in range(1, cfg["epochs"] + 1):
        model.train()
        order = np.random.permutation(len(tr))
        total = 0.0
        for start in range(0, len(order), cfg["batch"]):
            idx = order[start:start + cfg["batch"]]
            batch = il_model.collate([items["train"][i] for i in idx], [weights[i] for i in idx])
            opt.zero_grad()
            loss = il_model.loss(model, batch)
            loss.backward()
            opt.step()
            if model.residual and cfg["terminal_weight_min"] is not None:
                with torch.no_grad():
                    model.dg_weight[0].clamp_(min=cfg["terminal_weight_min"])
            total += loss.item() * len(idx)
        v = evaluate(model, va, items["val"])
        improved = best is None or v["nll"] < best["val"]["nll"]
        row = {"epoch": epoch, "train_loss": total / len(tr), **{"val_" + k: x for k, x in v.items()}}
        history.append(row)
        if improved:
            best = {"epoch": epoch, "val": v}
            best_state = {k: x.clone() for k, x in model.state_dict().items()}
            since_best = 0
        else:
            since_best += 1
        if epoch % log_every == 0 or improved or since_best > cfg["patience"]:
            print("epoch {:4d} {:5.1f}s  train loss {:.3f} | val {}{}".format(
                epoch, time.time() - t0, row["train_loss"], fmt(v), "  *" if improved else ""), file=log, flush=True)
        if since_best > cfg["patience"]:
            print("stopped: no improvement for {} epochs".format(cfg["patience"]), file=log)
            break

    model.load_state_dict(best_state)
    result = {"config": cfg, "best_epoch": best["epoch"], "val": best["val"],
              "train": evaluate(model, tr, items["train"]), "test": evaluate(model, te, items["test"])}
    print("best epoch {}: val {}".format(best["epoch"], fmt(best["val"])), file=log)
    if model.residual:
        result["dg_weight"] = model.dg_weight.tolist()
        print("  DynamicGap cost weights: terminal {:.3f}, clearance {:.3f}".format(*result["dg_weight"]), file=log)
    print("  operator: {} | model: {}".format(
        fmt(best["val"], ("operator_switch_rate", "operator_blip_rate")),
        fmt(best["val"], ("model_switch_rate", "model_blip_rate"))), file=log, flush=True)
    if out_dir:
        save(model, result, history, out_dir, data_dir)
        export_scorer(model, os.path.join(out_dir, "gap_scorer.json"))
    return model, result


def save(model, result, history, out_dir, data_dir):
    os.makedirs(out_dir, exist_ok=True)
    torch.save({"state_dict": model.state_dict(), "config": result["config"], "data": data_dir and os.path.abspath(data_dir)},
               os.path.join(out_dir, "model.pt"))
    with open(os.path.join(out_dir, "result.json"), "w") as f:
        json.dump(result, f, indent=2)
    with open(os.path.join(out_dir, "history.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(history[0]))
        w.writeheader()
        w.writerows(history)


def load(path):
    """Model saved by save() (model.pt), in eval mode."""
    ckpt = torch.load(path, map_location="cpu")
    sd = ckpt["state_dict"]
    # models from before the scene have no "scene" in their config
    cfg = dict(DEFAULTS, scene=False)
    cfg.update(ckpt["config"])
    model = build_model(cfg, il_model.norm_of(sd))
    model.load_state_dict(sd)
    return model.eval()


SCORER_FORMAT = "dynamic_gap_gap_scorer"
SCORER_FORMAT_VERSION = 2  # 2: the scene (scene, scene_features, scene_* tensors)


def export_scorer(model, path):
    """The model for the planner's C++ scorer (LearnedGapScorer): one JSON file
    with the feature lists, flags, caps and every tensor (row-major). The
    planner checks the feature lists against its own and refuses a mismatch."""
    model = model.eval()
    cfg = {
        "format": SCORER_FORMAT, "version": SCORER_FORMAT_VERSION,
        # the features the model was trained on (older models: a leading part of the list)
        "candidate_features": il_model.CANDIDATE_FEATURES[:len(model.cand_mean)],
        "context_features": il_model.CONTEXT_FEATURES,
        "scene_features": il_model.SCENE_FEATURES,
        "edge_match_rays": il_model.EDGE_MATCH_RAYS,
        "inputs": model.inputs,
        "network": model.network, "residual": model.residual, "attention": model.attention,
        "scene": model.scene,
        # one head count for both attention blocks
        "heads": model.mix.heads if getattr(model, "mix", None) is not None
        else model.read_scene.heads if model.scene else 0,
        "terminal_cost_cap": il_model.TERMINAL_COST_CAP,
        "clearance_cost_cap": il_model.CLEARANCE_COST_CAP,
        "masked_score": il_model.MASKED_SCORE,
        "tensors": {k: {"shape": list(v.shape), "data": v.detach().float().reshape(-1).tolist()}
                    for k, v in model.state_dict().items() if v.is_floating_point()},
    }
    with open(path, "w") as f:
        json.dump(cfg, f)


def without_dg_costs(model):
    """The same network without the linear term on DynamicGap's costs, for a
    model trained (residual on) on data without costs: there the term was zero
    in training, while the planner always has the costs and would apply it."""
    if not model.network:
        raise ValueError("a linear model is only the term on DynamicGap's costs: nothing is left without it")
    sd = {k: v for k, v in model.state_dict().items() if k != "dg_weight"}
    out = il_model.GapScorer(il_model.norm_of(sd), hidden=sd["cand_enc.0.weight"].shape[0], network=True,
                             residual=False, attention=model.attention,
                             heads=model.mix.heads if model.mix is not None else 4,
                             inputs=model.inputs, scene=model.scene)
    out.load_state_dict(sd)
    return out.eval()


def load_exported(path):
    """Model from an export_scorer() file, in eval mode."""
    with open(path) as f:
        cfg = json.load(f)
    if cfg.get("format") != SCORER_FORMAT:
        raise ValueError("{} is not a gap scorer export".format(path))
    state = {k: torch.tensor(v["data"], dtype=torch.float32).reshape(v["shape"]) for k, v in cfg["tensors"].items()}
    hidden = state["cand_enc.0.weight"].shape[0] if cfg["network"] else DEFAULTS["hidden"]
    model = il_model.GapScorer(il_model.norm_of(state), hidden=hidden, network=cfg["network"],
                               residual=cfg["residual"], attention=cfg["attention"], heads=cfg["heads"] or 4,
                               inputs=cfg["inputs"], scene=cfg.get("scene", False))
    missing, unexpected = model.load_state_dict(state, strict=False)
    if unexpected or set(missing) - {"cand_idx", "ctx_idx"}:
        raise ValueError("export does not match the model: missing {}, unexpected {}".format(missing, unexpected))
    return model.eval()


def export_onnx(model, path):
    b, n = 1, 3
    args = (torch.zeros(b, n, len(il_model.CANDIDATE_FEATURES)), torch.zeros(b, len(il_model.CONTEXT_FEATURES)),
            torch.zeros(b, n, 2), torch.zeros(b, n), torch.ones(b, n, dtype=torch.bool))
    torch.onnx.export(model, args, path, input_names=["cand", "ctx", "dg", "dg_avail", "mask"],
                      output_names=["scores"], opset_version=13,
                      dynamic_axes={k: {0: "batch", 1: "candidates"} for k in ("cand", "dg", "dg_avail", "mask")})

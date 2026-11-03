#!/usr/bin/env python3
"""Gap scorer (il_model.py) on synthetic examples. Needs torch, not ROS."""

import gzip
import io
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

import torch

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import il_model  # noqa: E402
import il_train  # noqa: E402


def candidate(rng, feasible=True, terminal=None, clearance=None, ids=(1, 2)):
    lt, rt = rng.uniform(0.2, 1.5), rng.uniform(-1.5, -0.2)
    return {
        "left_model_id": ids[0], "right_model_id": ids[1],
        "feasible": feasible, "shown": feasible, "tube_len": 1,
        "left_range": rng.uniform(1, 5), "left_theta": lt,
        "right_range": rng.uniform(1, 5), "right_theta": rt,
        "bearing": (lt + rt) / 2,
        "left_model": [rng.uniform(-3, 3) for _ in range(4)],
        "right_model": [rng.uniform(-3, 3) for _ in range(4)],
        "goal_rbt": [rng.uniform(0, 3), rng.uniform(-2, 2)],
        "terminal_rbt": [rng.uniform(0, 3), rng.uniform(-2, 2)] if feasible else [math.nan, math.nan],
        "path_rbt": [[rng.uniform(0, 3), rng.uniform(-2, 2)] for _ in range(rng.randint(1, 3))] if feasible else [],
        "dg_terminal_cost": terminal, "dg_clearance_cost": clearance,
    }


def example(rng, n=4, label=0, costs=False, episode=0, dg_pick=None):
    cands = [candidate(rng, feasible=(i < n - 1 or n == 1), ids=(i, i + 100),
                       terminal=rng.uniform(0, 4) if costs else None,
                       clearance=rng.uniform(0, 1) if costs else None) for i in range(n)]
    return {"session": "s", "episode": episode, "candidates": cands, "label": label, "dg_pick": dg_pick,
            "robot_vel": [rng.uniform(0, 1), 0.0, rng.uniform(-1, 1)],
            "goal": [rng.uniform(0, 10), rng.uniform(-5, 5)], "local_waypoint": [2.0, 0.0]}


def relabel(ex, key):
    """Label = the feasible candidate with the largest key(candidate)."""
    feasible = [i for i, c in enumerate(ex["candidates"]) if c["feasible"]]
    ex["label"] = max(feasible, key=lambda i: key(ex["candidates"][i]))
    return ex


def batch_of(examples):
    items = [il_model.featurize(ex) for ex in examples]
    return items, il_model.collate(items)


def fit(model, batch, steps=300, lr=0.01):
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    model.train()
    for _ in range(steps):
        opt.zero_grad()
        il_model.loss(model, batch).backward()
        opt.step()
    model.eval()


class FeaturesTest(unittest.TestCase):
    def test_shapes_and_mask(self):
        rng = random.Random(0)
        ex = example(rng, n=4, costs=True)
        ex["candidates"][1]["dg_clearance_cost"] = math.inf
        f = il_model.featurize(ex)
        self.assertEqual(f["cand"].shape, (4, len(il_model.CANDIDATE_FEATURES)))
        self.assertEqual(f["ctx"].shape, (len(il_model.CONTEXT_FEATURES),))
        self.assertEqual(f["mask"].tolist(), [True, True, True, False])
        self.assertTrue((f["cand"] == f["cand"]).all())  # no NaN from the infeasible candidate
        self.assertEqual(f["dg"][1, 1], il_model.CLEARANCE_COST_CAP)
        self.assertEqual(f["dg_avail"].tolist(), [1, 1, 1, 1])

    def test_costs_missing_before_schema_2(self):
        # terminal cost rebuilt from the trajectory end and the local waypoint, no clearance cost
        ex = example(random.Random(0), costs=False)
        f = il_model.featurize(ex)
        self.assertEqual(f["dg_avail"].tolist(), [1, 1, 1, 0])  # the infeasible one has no trajectory end
        self.assertFalse(f["dg_clearance"])
        self.assertEqual(f["dg"][:, 1].tolist(), [0, 0, 0, 0])
        end, wp = ex["candidates"][0]["terminal_rbt"], ex["local_waypoint"]
        self.assertAlmostEqual(float(f["dg"][0, 0]), il_model.Q_F * math.hypot(end[0] - wp[0], end[1] - wp[1]), 5)
        self.assertTrue(il_model.featurize(example(random.Random(0), costs=True))["dg_clearance"])

    def test_label_always_selectable(self):
        ex = example(random.Random(0), n=3, label=2)  # candidate 2 is infeasible
        self.assertTrue(il_model.featurize(ex)["mask"][2])

    def test_options_only(self):
        ex = example(random.Random(0), n=4, label=2)  # candidate 3 is infeasible
        ex["candidates"][0]["shown"] = False
        ex["dg_pick"] = 1
        out = il_model.options_only(ex)
        self.assertEqual(out["candidates"], [ex["candidates"][1], ex["candidates"][2]])
        self.assertEqual((out["label"], out["dg_pick"]), (1, 0))
        ex["dg_pick"] = 0  # DynamicGap's pick is not an option
        self.assertEqual(il_model.options_only(ex)["dg_pick"], -1)
        # the operator's gap stays
        ex["label"] = 3
        self.assertEqual(il_model.options_only(ex)["candidates"][-1], ex["candidates"][3])

    def test_cost_infinite(self):
        ex = example(random.Random(0), n=3, label=0)
        ex["candidates"][1]["dg_cost"] = math.inf
        col = il_model.CANDIDATE_FEATURES.index("cost_infinite")
        self.assertEqual(il_model.featurize(ex)["cand"][:, col].tolist(), [0, 1, 0])

    def test_model_on_fewer_features_still_runs(self):
        # a model trained before cost_infinite existed: statistics of the leading features only
        ex = example(random.Random(0), n=3, label=0)
        items = [il_model.featurize(ex)]
        norm = {k: v[:-1] if k.startswith("cand") else v for k, v in il_model.normalizer(items).items()}
        old = il_model.inputs_of(["edges", "ped_pos", "ped_vel", "robot_vel", "goal", "waypoint"])
        model = il_model.GapScorer(norm, inputs=old).eval()
        with torch.no_grad():
            s = il_model.scores(model, il_model.collate(items))[0]
        self.assertTrue(torch.isfinite(s).all())

    def test_options_only_same_scores(self):
        ex = example(random.Random(0), n=4, label=0)  # candidate 3 is infeasible
        items = [il_model.featurize(ex)]
        model = il_model.GapScorer(il_model.normalizer(items)).eval()
        kept = il_model.featurize(il_model.options_only(ex))
        with torch.no_grad():
            before = il_model.scores(model, il_model.collate(items))[0]
            after = il_model.scores(model, il_model.collate([kept]))[0]
        self.assertTrue(torch.allclose(before[:3], after, atol=1e-5))

    def test_decision_weights(self):
        rng = random.Random(0)
        exs = [example(rng, label=0, dg_pick=0), example(rng, label=0, dg_pick=1),
               example(rng, label=1, dg_pick=1), example(rng, label=1, episode=1)]
        self.assertEqual(il_model.decision_weights(exs, 2.0, 0.5), [1.0, 1.5, 3.0, 1.0])


class ScorerTest(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(0)
        rng = random.Random(1)
        self.examples = [example(rng, n=rng.randint(2, 6), costs=True) for _ in range(64)]
        self.items, self.batch = batch_of(self.examples)
        self.norm = il_model.normalizer(self.items)

    def test_masked_candidates_never_chosen(self):
        model = il_model.GapScorer(self.norm).eval()
        s = il_model.scores(model, self.batch)
        self.assertTrue((s[~self.batch["mask"]] == il_model.MASKED_SCORE).all())
        self.assertTrue(self.batch["mask"].gather(1, s.argmax(-1, keepdim=True)).all())

    def test_padding_and_order_do_not_matter(self):
        model = il_model.GapScorer(self.norm).eval()
        fit(model, self.batch, steps=20)  # move the head off zero
        alone = il_model.scores(model, il_model.collate(self.items[:1]))[0]
        padded = il_model.scores(model, self.batch)[0, :alone.shape[0]]
        self.assertTrue(torch.allclose(alone, padded, atol=1e-5))
        ex = self.examples[0]
        perm = list(reversed(range(len(ex["candidates"]))))
        flipped = dict(ex, candidates=[ex["candidates"][i] for i in perm], label=perm.index(ex["label"]))
        s = il_model.scores(model, il_model.collate([il_model.featurize(flipped)]))[0]
        self.assertTrue(torch.allclose(s, alone[perm], atol=1e-5))

    def test_network_starts_as_linear_model(self):
        linear = il_model.GapScorer(self.norm, network=False).eval()
        full = il_model.GapScorer(self.norm).eval()
        self.assertTrue(torch.allclose(il_model.scores(linear, self.batch), il_model.scores(full, self.batch)))

    def test_linear_model_learns_cost_weights(self):
        # the operator ignores clearance and minimises the terminal cost
        for ex in self.examples:
            relabel(ex, lambda c: -c["dg_terminal_cost"])
        _, batch = batch_of(self.examples)
        model = il_model.GapScorer(self.norm, network=False)
        fit(model, batch)
        self.assertEqual(il_model.top1(model, batch), 1.0)
        w = model.dg_weight.detach()
        self.assertGreater(w[0], 5 * w[1].abs())

    def test_network_learns_a_rule_without_costs(self):
        # no DynamicGap costs; the operator takes the widest gap
        rng = random.Random(2)
        width = lambda c: math.hypot(  # noqa: E731
            c["left_range"] * math.cos(c["left_theta"]) - c["right_range"] * math.cos(c["right_theta"]),
            c["left_range"] * math.sin(c["left_theta"]) - c["right_range"] * math.sin(c["right_theta"]))
        exs = [relabel(example(rng, n=rng.randint(2, 6)), width) for _ in range(256)]
        items, batch = batch_of(exs)
        for attention in (True, False):
            model = il_model.GapScorer(il_model.normalizer(items), attention=attention, dropout=0.0)
            fit(model, batch, steps=400, lr=0.005)
            self.assertGreater(il_model.top1(model, batch), 0.9, "attention={}".format(attention))

    def test_exports_to_onnx(self):
        model = il_model.GapScorer(self.norm).eval()
        b = self.batch
        torch.onnx.export(model, (b["cand"], b["ctx"], b["dg"], b["dg_avail"], b["mask"]), io.BytesIO(),
                          input_names=["cand", "ctx", "dg", "dg_avail", "mask"], output_names=["scores"],
                          dynamic_axes={k: {0: "batch", 1: "candidates"} for k in ("cand", "dg", "dg_avail", "mask")},
                          opset_version=13)


class TrainTest(unittest.TestCase):
    def test_unknown_identity(self):
        rng = random.Random(5)
        exs = [example(rng, n=2, episode=0) for _ in range(3)]
        for ex in exs:
            ex["label"] = 0
            ex["dg_pick"] = None
        ids = [(1, 2), (-1, -1), (3, 4)]
        for ex, (left, right) in zip(exs, ids):
            ex["candidates"][0]["left_model_id"], ex["candidates"][0]["right_model_id"] = left, right
        self.assertIsNone(il_model.gap_identity(exs[1]["candidates"][0]))
        # (1,2) -> unknown -> (3,4): a switch from the last known gap
        self.assertEqual([1.0, 1.0, 2.0], il_model.decision_weights(exs, 1.0, 0.0))
        for t, ex in enumerate(exs):
            ex["obs_tick"] = t
        # a gap without identity is a gap of its own: two switches
        self.assertEqual(1.0, il_train.stickiness(exs, [0, 0, 0])["switch_rate"])
        # no model ids at all (older datasets): unknown
        bare = [dict(ex, candidates=[{k: v for k, v in c.items() if not k.endswith("model_id")}
                                     for c in ex["candidates"]]) for ex in exs]
        self.assertEqual({"switch_rate": None, "blip_rate": None}, il_train.stickiness(bare, [0, 0, 0]))
        # (1,2) -> unknown -> (1,2): a blip
        exs[2]["candidates"][0]["left_model_id"], exs[2]["candidates"][0]["right_model_id"] = 1, 2
        self.assertEqual(0.5, il_train.stickiness(exs, [0, 0, 0])["blip_rate"])
        # ids recorded, but every pick is a gap without identity: switches every step
        for ex in exs:
            ex["candidates"][0]["left_model_id"] = -1
        self.assertEqual(1.0, il_train.stickiness(exs, [0, 0, 0])["switch_rate"])

    def test_stickiness(self):
        rng = random.Random(0)
        exs = [example(rng, n=3, label=l) for l in (0, 0, 1, 0)]
        for t, ex in enumerate(exs):
            ex["obs_tick"] = t
        # A A B A: 2 switches and 1 blip in 3 steps
        st = il_train.stickiness(exs, [ex["label"] for ex in exs])
        self.assertAlmostEqual(st["switch_rate"], 2 / 3)
        self.assertAlmostEqual(st["blip_rate"], 1 / 3)

    def test_train_save_load(self):
        rng = random.Random(3)
        splits = []
        for n in (128, 32, 16):
            exs = [example(rng, n=rng.randint(2, 5), episode=i // 8) for i in range(n)]
            for t, ex in enumerate(exs):
                ex["obs_tick"] = t
                relabel(ex, lambda c: c["left_range"])
            splits.append(exs)
        out = tempfile.mkdtemp()
        try:
            model, result = il_train.train(None, {"epochs": 3, "groups": ["edges", "goal"]}, out,
                                           log=io.StringIO(), data=splits)
            self.assertEqual(set(result), {"config", "best_epoch", "val", "train", "test", "dg_weight"})
            # rebuilt terminal costs only: the clearance weight stays out
            self.assertEqual(result["dg_weight"][1], 0.0)
            loaded = il_train.load(os.path.join(out, "model.pt"))
            batch = il_model.collate([il_model.featurize(ex) for ex in splits[1]])
            self.assertTrue(torch.allclose(il_model.scores(model.eval(), batch), il_model.scores(loaded, batch)))
            self.assertEqual(loaded.inputs, il_model.inputs_of(["edges", "goal"]))
            # the planner's export gives the same scores
            il_train.export_scorer(model, os.path.join(out, "gap_scorer.json"))
            exported = il_train.load_exported(os.path.join(out, "gap_scorer.json"))
            self.assertTrue(torch.allclose(il_model.scores(model, batch), il_model.scores(exported, batch)))
            self.assertTrue(os.path.exists(os.path.join(out, "gap_scorer.json")))
        finally:
            shutil.rmtree(out)

    def _splits(self, sizes):
        rng = random.Random(4)
        splits = []
        for n in sizes:
            exs = [example(rng, n=rng.randint(2, 5), episode=i // 8) for i in range(n)]
            for t, ex in enumerate(exs):
                ex["obs_tick"] = t
                relabel(ex, lambda c: c["left_range"])
            splits.append(exs)
        return splits

    def test_linear_model_has_no_network_without_costs(self):
        norm = {"cand_mean": torch.zeros(len(il_model.CANDIDATE_FEATURES)), "cand_std": torch.ones(len(il_model.CANDIDATE_FEATURES)),
                "ctx_mean": torch.zeros(len(il_model.CONTEXT_FEATURES)), "ctx_std": torch.ones(len(il_model.CONTEXT_FEATURES))}
        with self.assertRaisesRegex(ValueError, "linear"):
            il_train.without_dg_costs(il_model.GapScorer(norm, network=False, residual=True))

    def test_train_needs_a_validation_split(self):
        with self.assertRaisesRegex(ValueError, "val split"):
            il_train.train(None, {"epochs": 1}, log=io.StringIO(), data=self._splits((16, 0, 0)))

    def test_ablation_runs(self):
        tmp = tempfile.mkdtemp()
        try:
            for split, exs in zip(("train", "val", "test"), self._splits((32, 16, 8))):
                with gzip.open(os.path.join(tmp, split + ".jsonl.gz"), "wt") as f:
                    f.writelines(json.dumps(ex) + "\n" for ex in exs)
            out = os.path.join(tmp, "out")
            subprocess.run([sys.executable, os.path.join(os.path.dirname(il_train.__file__), "il_ablate"),
                            "--data", tmp, "--out", out, "--seeds", "1", "--epochs", "1", "--only", "linear"],
                           check=True, stdout=subprocess.DEVNULL)
            self.assertTrue(os.path.exists(os.path.join(out, "summary.csv")))
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()

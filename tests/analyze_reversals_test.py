import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("analysis", ROOT / "tools/analyze_reversals.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class ReversalAnalysis(unittest.TestCase):
    def test_actual_bad_sample_precedes_normal_reversal_and_is_not_removed(self):
        report = analysis.analyze(*analysis.read_csv(ROOT / "tests/fixtures/experiment-1-overshoot.csv"))
        self.assertEqual(report["worst_excursion_reversal"], 7)
        bad = report["reversals"][6]
        self.assertEqual(bad["peak_sample"]["epos_counts"], 18023)
        self.assertEqual(bad["peak_sample"]["receipt_ms"], 3478)
        self.assertAlmostEqual(bad["max_excursion_um"], 17.846)
        self.assertEqual(bad["queue_to_complete_ms"], [1, 1, 1])
        self.assertEqual(bad["total_reversal_ms"], 3)
        self.assertEqual(bad["stat_values"], [8545])
        self.assertEqual(bad["classification"], "ambiguous")
        self.assertIsNone(bad["epos_at_exact_extremum"])
        self.assertEqual(len(report["reversals"]), 9)
        self.assertEqual(len(bad["large_receipt_displacements"]), 2)
        self.assertIn("direction completion slower than peers", report["reversals"][1]["correlations"])

    def test_injected_ordering_violation_is_distinguished_from_missing_evidence(self):
        meta, rows = analysis.read_csv(ROOT / "tests/fixtures/experiment-1-overshoot.csv")
        meta["reversals"][6]["commands"][4][2] = 3500
        bad = analysis.analyze(meta, rows)["reversals"][6]
        self.assertEqual(bad["classification"], "confirmed command anomaly")
        meta["reversals"][6]["commands"][5][3] = None
        bad = analysis.analyze(meta, rows)["reversals"][6]
        self.assertEqual(bad["classification"], "ambiguous")
        self.assertIsNone(bad["total_reversal_ms"])


if __name__ == "__main__":
    unittest.main()

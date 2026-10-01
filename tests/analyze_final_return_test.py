import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
spec = importlib.util.spec_from_file_location("final_analysis", ROOT / "tools/analyze_final_return.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class FinalReturnAnalysis(unittest.TestCase):
    def test_legacy_run_does_not_invent_missing_completion_feedback(self):
        report = analysis.analyze(*analysis.read_csv(ROOT / "tests/fixtures/experiment-1-overshoot.csv"))
        self.assertEqual(report["source_outcome"], "completed")
        self.assertEqual(report["theoretical_boundary_ms"], 5000)
        self.assertEqual(report["center_target_counts"], 17666)
        self.assertEqual(report["return_speed_um_s_from_config"], 63)
        self.assertEqual(report["nominal_travel_ms_from_boundary_cache"], 171)
        last = report["last_logged_feedback"]
        self.assertEqual(last["epos_counts"], 17652)
        self.assertEqual(last["epos_receipt_ms"], 5849)
        self.assertEqual(last["error_counts"], -14)
        self.assertEqual(last["error_um"], -1.092)
        self.assertFalse(last["position_reached"])
        self.assertIsNone(report["final_return_diagnostics"])
        self.assertIn("actual final error", report["unavailable_in_legacy_export"])
        self.assertEqual(len(report["feedback"]), 44)

    def test_terminal_evidence_is_preserved_separately_from_last_logger_tick(self):
        metadata, rows = analysis.read_csv(ROOT / "tests/fixtures/experiment-1-overshoot.csv")
        metadata["final_return"] = {"error_counts": -2, "error_um": -.156,
                                    "finished_ms": 56789, "expired": False}
        report = analysis.analyze(metadata, rows)
        self.assertEqual(report["final_return_diagnostics"], metadata["final_return"])
        self.assertEqual(report["last_logged_feedback"]["error_counts"], -14)
        self.assertEqual(report["unavailable_in_legacy_export"], [])


if __name__ == "__main__":
    unittest.main()

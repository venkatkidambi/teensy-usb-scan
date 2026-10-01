#!/usr/bin/env python3
"""Reconstruct final-return evidence without inferring missing TX/echo events."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from analyze_reversals import read_csv


def analyze(metadata, rows):
    config = metadata["config"]
    if config["kind"] != "sinusoidal":
        raise ValueError("Final center return applies to sinusoidal runs")
    exact_boundary = config["cycles"] * 1_000_000 / config["frequency_mhz"]
    boundary = math.ceil(exact_boundary)
    center_nm = config["center_nm"]
    target = ((abs(center_nm) + 39) // 78) * (-1 if center_nm < 0 else 1)
    speed = math.floor(2 * math.pi * config["amplitude_nm"] * config["frequency_mhz"] / 1e6 + .5)
    evidence = metadata.get("final_return")
    samples = []
    for row in rows:
        elapsed = round(float(row["time_s"]) * 1000)
        if elapsed < boundary:
            continue
        stat = int(row["stat"])
        error = int(row["epos_counts"]) - target
        ticks = int(row["controller_ticks"])
        age = int(row["controller_age_ms"])
        samples.append({
            "log_ms": elapsed, "epos_counts": int(row["epos_counts"]),
            "epos_receipt_ms": elapsed - int(row["epos_age_ms"]),
            "stat": stat, "stat_receipt_ms": elapsed - int(row["stat_age_ms"]),
            "position_reached": bool(stat & (1 << 10)),
            "controller_ticks": ticks if age != 2**32 - 1 else None,
            "controller_receipt_ms": elapsed - age if age != 2**32 - 1 else None,
            "error_counts": error, "error_um": round(error * .078, 3),
        })
    transitions = []
    for sample in samples:
        if not transitions or transitions[-1]["stat"] != sample["stat"]:
            transitions.append({k: sample[k] for k in ["stat_receipt_ms", "stat", "position_reached"]})
    distance = abs(samples[0]["epos_counts"] - target) * 78 if samples else None
    nominal = math.ceil(distance / speed) if distance is not None and speed else None
    return {
        "source_outcome": metadata["outcome"], "source_reason": metadata["reason"],
        "started_host_ms": metadata["started_ms"], "completed_or_faulted_ms": metadata["elapsed_ms"],
        "theoretical_boundary_ms": exact_boundary, "first_representable_boundary_ms": boundary,
        "theoretical_final_position_um": (config["center_nm"] - config["amplitude_nm"]) / 1000,
        "theoretical_final_velocity_um_s": 0, "center_target_counts": target,
        "return_speed_um_s_from_config": speed, "nominal_travel_ms_from_boundary_cache": nominal,
        "retained_deadline_budget_ms": metadata["move_timeout_ms"],
        "last_logged_feedback": samples[-1] if samples else None,
        "logged_stat_transitions": transitions,
        "final_return_diagnostics": evidence,
        "unavailable_in_legacy_export": [] if evidence else [
            "STOP scheduled/queued/completed times", "waveform TX/intent pending at STOP",
            "DPOS scheduled/queued/completed times", "received DPOS echo",
            "terminal EPOS/STAT/TIME at completion", "terminal Position Reached transition",
            "PTOL/PTO2/TOUT/DLAY readbacks", "actual final error", "deadline expiration state",
        ],
        "feedback": samples,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(*read_csv(args.input))
    result["source_sha256"] = hashlib.sha256(args.input.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2) + "\n")

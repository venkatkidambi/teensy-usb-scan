#!/usr/bin/env python3
"""Compare raw reversal evidence in an exported experiment CSV. Standard library only.

Usage: python3 tools/analyze_reversals.py input.csv --output analysis.json
All EPOS times are host receipt times; TIME is an unpaired latest-value cache.
No smoothing, interpolation, deletion of samples, or physical-time correction.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median


def read_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.reader(stream)
        metadata = json.loads(next(reader)[1])
        fields = next(reader)
        rows = [dict(zip(fields, row)) for row in reader]
    return metadata, rows


def analyze(metadata, rows):
    config = metadata["config"]
    if config["kind"] != "sinusoidal":
        return {"reversals": [], "reason": "quasi-static experiment"}
    center = config["center_nm"] / 1000
    amplitude = config["amplitude_nm"] / 1000
    frequency = config["frequency_mhz"] / 1000
    peak_speed = 2 * math.pi * frequency * amplitude
    points = {}
    for row in rows:
        generation = int(row["epos_generation"])
        points.setdefault(generation, {
            "receipt_ms": float(row["epos_host_receipt_s"]) * 1000,
            "epos_counts": int(row["epos_counts"]),
            "generation": generation,
            "receipt_age_ms": int(row["epos_age_ms"]),
            "stat": int(row["stat"]),
            "stat_age_ms": int(row["stat_age_ms"]),
            "controller_ticks": int(row["controller_ticks"]),
            "controller_age_ms": int(row["controller_age_ms"]),
        })
    # New firmware captures additional raw observations between logger samples.
    for reversal in metadata.get("reversals", []):
        for s in reversal.get("detail", {}).get("window", []):
            value = s[1] if s[1] < 2**31 else s[1] - 2**32
            points[s[5]] = {
                "receipt_ms": s[0] - s[3], "epos_counts": value,
                "generation": s[5], "receipt_age_ms": s[3],
                "stat": s[2], "stat_age_ms": s[4],
                "controller_ticks": s[6], "controller_age_ms": s[7],
                "tx_busy": bool(s[10]) if len(s) > 10 else None,
            }
    points = sorted(points.values(), key=lambda p: p["receipt_ms"])
    results = []
    for index, reversal in enumerate(metadata.get("reversals", [])):
        detail = reversal.get("detail", {})
        crossing = (index + 1) * 500 / frequency
        window = metadata.get("reversal_window_ms", 40)
        nearby = [p for p in points if abs(p["receipt_ms"] - crossing) <= window]
        sign = -reversal["direction"]
        extremum = center + sign * amplitude
        commands = reversal["commands"]
        stages = commands[3:6]
        zero, direction, resume = stages
        complete = all(c is not None and c[3] is not None for c in stages)
        flags = []
        if complete:
            if not (zero[1] == 0 and zero[3] <= direction[2]
                    and direction[3] <= resume[2] and direction[1] == reversal["direction"]):
                flags.append("command ordering/direction violation")
            for c in commands[:3]:
                if c and c[3] is not None and c[3] > zero[2]:
                    flags.append("pre-reversal ordinary command completed after zero queued")
            expected = math.floor(abs(peak_speed * math.sin(2 * math.pi * frequency * resume[2] / 1000)) + 0.5)
            if resume[1] != expected:
                flags.append("resume SSPD differs from actual-queue-time theory")
        else:
            flags.append("incomplete command evidence")
        signed_excursions = [(p["epos_counts"] * .078 - extremum) * sign for p in nearby]
        peak = nearby[max(range(len(nearby)), key=lambda i: signed_excursions[i])] if nearby else None
        nearest = min(nearby, key=lambda p: abs(p["receipt_ms"] - crossing)) if nearby else None
        maximum_error = max((abs(p["epos_counts"] * .078 -
                            (center - amplitude * math.cos(2 * math.pi * frequency * p["receipt_ms"] / 1000)))
                             for p in nearby), default=None)
        if reversal.get("max_error_counts") is not None:
            maximum_error = max(maximum_error or 0, reversal["max_error_counts"] * .078)
        if reversal.get("rx_backlog_polls", 0):
            flags.append("RX backlog observed")
        if detail.get("coalesced", 0):
            flags.append("coalesced updates during reversal")
        if detail.get("lateness_ms", 0):
            flags.append("scheduler lateness")
        stats = sorted({p["stat"] for p in nearby})
        fault_mask = sum(1 << bit for bit in [2, 3, 12, 16, 18, 21])
        if any(stat & fault_mask for stat in stats):
            flags.append("controller fault STAT")
        if len(stats) > 1 or detail.get("stat_changes", 0):
            flags.append("STAT change")
        intervals = [b["receipt_ms"]-a["receipt_ms"] for a,b in zip(nearby, nearby[1:])]
        generation_gaps = sum(max(0, ((b["generation"]-a["generation"]) % 2**32)-1)
                              for a,b in zip(nearby, nearby[1:]))
        if any(dt <= 1 for dt in intervals):
            flags.append("EPOS receipts clustered; generation timing unknown")
        if generation_gaps:
            flags.append("EPOS publications missing from retained window")
        jumps = []
        for previous, current in zip(points, points[1:]):
            if current not in nearby:
                continue
            dt = current["receipt_ms"] - previous["receipt_ms"]
            delta = (current["epos_counts"] - previous["epos_counts"]) * .078
            if dt <= 0 or abs(delta) > peak_speed * dt / 1000 + .156:
                jumps.append({"from_ms": previous["receipt_ms"], "to_ms": current["receipt_ms"],
                              "delta_um": round(delta, 6),
                              "receipt_slope_um_s": delta * 1000 / dt if dt > 0 else None})
        if jumps:
            flags.append("EPOS changes exceed commanded peak-speed displacement; not proof of physical speed")
        results.append({
            "index": index + 1, "theoretical_crossing_ms": crossing,
            "observed_transition_ms": reversal["crossing_ms"],
            "direction_before": -reversal["direction"], "direction_after": reversal["direction"],
            "theoretical_extremum_um": extremum, "theoretical_velocity_um_s": 0,
            "epos_at_exact_extremum": next((p for p in nearby if abs(p["receipt_ms"]-crossing) < 1e-6), None),
            "nearest_epos": nearest,
            "nearest_receipt_offset_ms": nearest["receipt_ms"] - crossing if nearest else None,
            "max_excursion_um": max(0, max(signed_excursions, default=0), detail.get("excursion_counts", 0) * .078),
            "peak_sample": peak, "max_receipt_trajectory_error_um": maximum_error,
            "command_timeline": commands, "command_tokens": reversal.get("command_tokens"),
            "scheduled_ms": detail.get("scheduled"), "state_transitions_plus_one_ms": detail.get("transitions"),
            "queue_to_complete_ms": [c[3] - c[2] if c and c[3] is not None else None for c in stages],
            "crossing_to_complete_ms": [c[3] - crossing if c and c[3] is not None else None for c in stages],
            "total_reversal_ms": resume[3] - crossing if resume and resume[3] is not None else None,
            "resumed_speed_um_s": resume[1] if resume else None,
            "max_tx_busy_ms": detail.get("max_busy_ms"), "coalesced_updates": detail.get("coalesced"),
            "rx_backlog_polls": reversal.get("rx_backlog_polls"), "stat_values": stats,
            "max_receipt_age_ms": max((p["receipt_age_ms"] for p in nearby), default=None),
            "window_dropped": detail.get("dropped"), "feedback_window": nearby,
            "receipt_intervals_ms": intervals, "unobserved_generations": generation_gaps,
            "large_receipt_displacements": jumps, "correlations": flags,
            "classification": "confirmed command anomaly" if "command ordering/direction violation" in flags else "ambiguous",
        })
    for stage in range(3):
        latencies = [r["queue_to_complete_ms"][stage] for r in results if r["queue_to_complete_ms"][stage] is not None]
        typical = median(latencies) if latencies else None
        if typical is not None:
            for r in results:
                latency = r["queue_to_complete_ms"][stage]
                if latency is not None and latency > max(typical * 3, typical + 2):
                    r["correlations"].append(["zero", "direction", "resume"][stage] + " completion slower than peers")
    return {"config": config, "timing_basis": "host receipt and USB transport completion; not actuator timing",
            "controller_time_basis": "latest independent TIME cache; not paired to EPOS",
            "peak_commanded_speed_um_s": peak_speed, "reversals": results,
            "worst_excursion_reversal": max(results, key=lambda r: r["max_excursion_um"])["index"] if results else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = analyze(*read_csv(args.csv))
    report["source_sha256"] = hashlib.sha256(args.csv.read_bytes()).hexdigest()
    encoded = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(encoded)
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()

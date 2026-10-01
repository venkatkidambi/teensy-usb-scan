#!/usr/bin/env python3
"""Compose the supplied LPS/probe and probe/force XLSX calibrations (stdlib only).

Workbook cells are data only; formulas/macros are never executed. Cached numeric
values are checked against the probe table's ordinary least-squares fit.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET
import zipfile

NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}


def cells(path):
    with zipfile.ZipFile(path) as workbook:
        sheet = ET.fromstring(workbook.read("xl/worksheets/sheet1.xml"))
        return {
            cell.attrib["r"]: float(cell.find("m:v", NS).text)
            for cell in sheet.findall(".//m:sheetData/m:row/m:c", NS)
            if cell.get("t") not in ("s", "str", "inlineStr")
            and cell.find("m:v", NS) is not None
        }


def fit(points):
    xmean = statistics.mean(x for x, _ in points)
    ymean = statistics.mean(y for _, y in points)
    slope = sum((x-xmean)*(y-ymean) for x, y in points) / sum(
        (x-xmean)**2 for x, _ in points
    )
    intercept = ymean - slope*xmean
    residual = sum((y-(slope*x+intercept))**2 for x, y in points)
    return {
        "slope": slope,
        "intercept": intercept,
        "r_squared": 1-residual/sum((y-ymean)**2 for _, y in points),
        "rmse": math.sqrt(residual/len(points)),
    }


def calibrate(lps_path, probe_path):
    lps_cells, probe_cells = cells(lps_path), cells(probe_path)
    pairs = [[int(lps_cells[f"E{r}"]), int(lps_cells[f"F{r}"])] for r in range(2, 52)]
    probe = [[int(probe_cells[f"A{r}"]), probe_cells[f"D{r}"]] for r in range(2, 13)]
    lps_fit, probe_fit = fit(pairs), fit(probe)
    for key, cell in [("slope", "C17"), ("intercept", "C18")]:
        if not math.isclose(probe_fit[key], probe_cells[cell], rel_tol=1e-12):
            raise ValueError(f"Probe {cell} does not match the sensitivity table")
        probe_fit[key] = probe_cells[cell]
    slope = lps_fit["slope"] * probe_fit["slope"]
    intercept = lps_fit["intercept"] * probe_fit["slope"] + probe_fit["intercept"]
    lps_range = [min(x for x, _ in pairs), max(x for x, _ in pairs)]
    probe_range = [min(x for x, _ in probe), max(x for x, _ in probe)]
    supported = [max(lps_range[0], (probe_range[0]-lps_fit["intercept"])/lps_fit["slope"]),
                 min(lps_range[1], (probe_range[1]-lps_fit["intercept"])/lps_fit["slope"])]
    return {
        "id": "lps-probe-force-2026-08-27-v1",
        "method": "OLS probe counts versus LPS counts; compose with workbook probe-to-newtons OLS. No tare or clamping.",
        "sources": [
            {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "cells": region}
            for path, region in [(lps_path, "results!E2:F51 (com6_value=LPS, com8_value=probe)"),
                                 (probe_path, "Sheet1!A2:A12,D2:D12; slope C17, intercept C18")]
        ],
        "lps_to_probe": lps_fit,
        "probe_to_force_n": probe_fit,
        "lps_to_force_n": {"slope": slope, "intercept": intercept},
        "measured_lps_range": lps_range,
        "measured_probe_range": probe_range,
        "supported_lps_range": supported,
        "lps_fit_rmse_mn": lps_fit["rmse"]*probe_fit["slope"]*1000,
        "observations": {"lps_counts_probe_counts": pairs, "probe_counts_force_n": probe},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lps_workbook", type=Path)
    parser.add_argument("probe_workbook", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/"desktop/calibration/lps-force.json")
    args = parser.parse_args()
    result = calibrate(args.lps_workbook, args.probe_workbook)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: result[key] for key in ["lps_to_force_n", "supported_lps_range", "lps_fit_rmse_mn"]}, indent=2))

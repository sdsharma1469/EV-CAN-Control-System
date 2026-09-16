#!/usr/bin/env python3
"""Run deterministic EV CAN simulator scenarios and validate expected states."""

from __future__ import annotations

import argparse
import csv
import io
import subprocess
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioExpectation:
    name: str
    required_state: str
    forbidden_state: str | None = None


SCENARIOS = [
    ScenarioExpectation("nominal", "READY", "FAULT"),
    ScenarioExpectation("slip", "DERATE", "FAULT"),
    ScenarioExpectation("pedal-conflict", "FAULT"),
    ScenarioExpectation("pedal-timeout", "FAULT"),
    ScenarioExpectation("counter-freeze", "FAULT"),
]


def run_scenario(binary: str, scenario: str) -> list[dict[str, str]]:
    result = subprocess.run(
        [binary, "--scenario", scenario],
        check=True,
        capture_output=True,
        text=True,
    )
    return list(csv.DictReader(io.StringIO(result.stdout)))


def validate(rows: list[dict[str, str]], expectation: ScenarioExpectation) -> tuple[bool, str]:
    if not rows:
        return False, "simulator produced no CSV rows"

    states = {row["state"] for row in rows}
    if expectation.required_state not in states:
        return False, f"expected {expectation.required_state}, observed {sorted(states)}"
    if expectation.forbidden_state and expectation.forbidden_state in states:
        return False, f"unexpected {expectation.forbidden_state}, observed {sorted(states)}"

    if expectation.name == "slip":
        derated = [row for row in rows if row["state"] == "DERATE"]
        if not any(float(row["slip_ratio"]) > 0.08 for row in derated):
            return False, "DERATE was not associated with slip above threshold"

    if expectation.name in {"pedal-conflict", "counter-freeze"}:
        fault_rows = [row for row in rows if row["state"] == "FAULT"]
        if not fault_rows or float(fault_rows[-1]["torque_nm"]) != 0.0:
            return False, "fault did not command zero torque"

    return True, f"observed states {sorted(states)}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="./build/ev_can_sim")
    args = parser.parse_args()

    failures = 0
    for expectation in SCENARIOS:
        try:
            rows = run_scenario(args.binary, expectation.name)
            ok, detail = validate(rows, expectation)
        except (subprocess.CalledProcessError, OSError, KeyError, ValueError) as exc:
            ok, detail = False, str(exc)

        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {expectation.name}: {detail}")
        failures += int(not ok)

    if failures:
        print(f"\n{failures} scenario(s) failed.")
        return 1

    print("\nAll fault-injection scenarios passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Characterize direct observation selection through the BagBuilder CLI."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def export(binary, fixture, bag, output, options=()):
    """Export a fixture with the existing legacy CLI's direct observation mode."""
    completed = subprocess.run(
        [
            str(binary), "--config", str(fixture / "demo_config.yaml"),
            "--bag", str(fixture / bag), "--output", str(output),
            "--map-id", "fixture", "--geometry-revision", "v1",
            "--map-basis", "Fixture TF", *options,
        ],
        capture_output=True, text=True, timeout=30, check=True,
    )
    return json.loads(completed.stdout)


def rows(path):
    """Read JSONL records in their published order."""
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def check_observations(report, observations, seconds, prepared_pairs):
    """Check original timestamps, retained evidence and the CLI counters."""
    stamps = [{"sec": second, "nanosec": 0} for second in seconds]
    assert [row["rgb_stamp"] for row in observations] == stamps, observations
    assert [row["depth_stamp"] for row in observations] == stamps, observations
    assert all(row["pending_reasons"] == ["capture_sync_unverified"] for row in observations)
    assert report["pairs"] == 2 and report["prepared_pairs"] == prepared_pairs, report
    assert report["observations"] == report["pending_observations"] == len(stamps), report


def main():
    """Compare unfiltered, sampled, and damaged-depth exports."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--fixture-pointer", required=True, type=Path)
    args = parser.parse_args()
    fixture = Path((args.fixture_pointer / "latest.txt").read_text(encoding="utf-8").strip())
    binary = args.binary.resolve()
    with tempfile.TemporaryDirectory(prefix="bag-observation-selection-") as directory:
        root = Path(directory)
        report = export(binary, fixture, "normal_bag", root / "all")
        observations = rows(root / "all/observations.jsonl")
        check_observations(report, observations, [10, 11], 2)
        observation_ids = [row["id"] for row in observations]
        assert len(set(observation_ids)) == 2

        sampled = export(binary, fixture, "normal_bag", root / "sampled", ["--distance-m", "1000"])
        selected = rows(root / "sampled/observations.jsonl")
        check_observations(sampled, selected, [10], 2)
        assert [row["id"] for row in selected] == observation_ids[:1]
        assert rows(root / "sampled/evidence/issues.jsonl") == rows(root / "all/evidence/issues.jsonl")

        damaged = export(binary, fixture, "bad_depth_compressed_bag", root / "damaged")
        surviving = rows(root / "damaged/observations.jsonl")
        check_observations(damaged, surviving, [11], 1)
        issues = rows(root / "damaged/evidence/issues.jsonl")
        assert [issue["code"] for issue in issues] == [
            "deserialize_failed", "invalid_compressed_payload", "pair_pending",
            "missing_rgbd_media", "outside_tolerance",
        ], issues
        assert issues[2]["pair_id"] == issues[3]["pair_id"] != surviving[0]["pair_id"]
        assert damaged["issue_count"] == len(issues)
    print("BagBuilder direct observation selection characterization passed")


if __name__ == "__main__":
    main()

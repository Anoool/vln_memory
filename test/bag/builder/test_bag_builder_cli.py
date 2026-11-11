"""Characterize the existing BagBuilder CLI using the ROS CTest fixture."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile


USAGE = (
    "Usage: ros2 run vln_memory bag_builder --config config/vln_memory.yaml\n"
    "  Legacy preparation: --config config/camera_record.yaml --format preparation\n"
)
MISSING_CONFIG = "Missing required option: --config\n"
REPORT_FIELDS = {
    "status", "code", "message", "output_directory", "work_directory",
    "rgb_frames", "depth_frames", "decoded_frames", "observations",
    "pending_observations", "camera_models", "pairs", "prepared_pairs",
    "issue_count", "timestamp_match_policy", "sync_tolerance_ns",
    "timestamp_match_status", "capture_sync_status",
}


def run_cli(binary, arguments):
    """Capture both output streams and the status from one CLI invocation."""
    return subprocess.run(
        [str(binary), *arguments], capture_output=True, text=True, timeout=30, check=False
    )


def check_arguments(binary):
    """Lock the parser's scan order and the output stream for each exit."""
    cases = [
        (["--help"], 0, USAGE, ""),
        (["-h"], 0, USAGE, ""),
        (["--help", "--unknown"], 0, USAGE, ""),
        (["--unknown", "--help"], 2, USAGE, ""),
        (["--config"], 2, USAGE, ""),
        (["--config", "first", "--config", "second"], 2, USAGE, ""),
        ([], 2, "", MISSING_CONFIG),
        (["--config", ""], 2, "", MISSING_CONFIG),
        (["--bag", "--help"], 2, "", MISSING_CONFIG),
    ]
    for arguments, code, stdout, stderr in cases:
        result = run_cli(binary, arguments)
        actual = (result.returncode, result.stdout, result.stderr)
        assert actual == (code, stdout, stderr), (arguments, actual)


def read_report(result, expected_exit):
    """Read the CLI report and check its shape, formatting, and effective options."""
    # rosbag2 emits timestamped diagnostics on stderr while reading the fixture.
    assert result.returncode == expected_exit, result
    report = json.loads(result.stdout)
    assert report.keys() == REPORT_FIELDS, report
    assert result.stdout == json.dumps(report, indent=2, sort_keys=True) + "\n", result.stdout
    assert report["sync_tolerance_ns"] == "30000000", report
    assert report["timestamp_match_policy"] == "rgb_reference_monotonic_nearest_one_to_one_v1", report
    assert report["capture_sync_status"] == "unknown", report
    return report


def check_preparation_report(binary, fixture):
    """Check the legacy CLI override and its report when publication is refused."""
    with tempfile.TemporaryDirectory(prefix="bag-builder-cli-") as directory:
        output = Path(directory) / "preparation"
        arguments = [
            "--config", str(fixture / "demo_config.yaml"), "--format", "preparation",
            "--output", str(output), "--sync-tolerance-ms", "30",
        ]
        report = read_report(run_cli(binary, arguments), 0)
        assert report["code"] == "ok" and report["output_directory"] == str(output), report
        assert report["pairs"] == 2, report
        manifest = output / "bag_manifest.json"
        published = manifest.read_bytes()

        failed = read_report(run_cli(binary, arguments), 1)
        assert failed["status"] == "failed" and failed["code"] == "output_exists", failed
        assert manifest.read_bytes() == published


def main():
    """Run the characterizations against the supplied BagBuilder binary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--fixture-pointer", required=True, type=Path)
    args = parser.parse_args()
    fixture = Path((args.fixture_pointer / "latest.txt").read_text(encoding="utf-8").strip())
    binary = args.binary.resolve()
    check_arguments(binary)
    check_preparation_report(binary, fixture)
    print("BagBuilder CLI characterization passed")


if __name__ == "__main__":
    main()

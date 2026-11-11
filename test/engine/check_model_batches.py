"""Check the installed-style batch command against a scripted local Responses server."""

import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import sqlite3
import subprocess
import tempfile
import threading


def message(text):
  """Return a completed assistant message."""
  return {"type": "message", "role": "assistant", "status": "completed",
          "content": [{"type": "output_text", "text": text}]}


def call_item(call_id, name, arguments):
  """Create the same function item shape returned by Responses."""
  return {"type": "function_call", "call_id": call_id, "name": name,
          "arguments": json.dumps(arguments), "status": "completed"}


def run_recovery(binary, dataset, config_path):
  """Resume after a failed receipt insert, HTTP failure, and tool round exhaustion."""
  database = dataset / "entities.sqlite3"
  with (dataset / "observations.jsonl").open() as stream:
    observations = [json.loads(line) for line in stream]
  ids = [item["id"] for item in observations]
  assert len(ids) == 2

  def create(label, obs_id):
    """Create a located object from a known depth pixel."""
    return {"kind": "object", "semantic_label": label, "add_observations": [
        {"observation_id": obs_id, "selector": {"type": "point", "image": "depth", "pixel_xy": [0, 0]}}]}

  first = call_item("call_01", "create_entity", create("first", ids[0]))
  second = call_item("call_02", "create_entity", create("second", ids[1]))
  reasoning = {"type": "reasoning", "id": "rs_fixture", "summary": [], "encrypted_content": "fixture"}
  posted = []
  authorization = []
  key_file = config_path.with_name("api_key.yaml")
  replies = [(200, [reasoning, first, second]), (503, []), (200, []), (200, [message("Done")])]

  class Handler(BaseHTTPRequestHandler):
    """Capture the exact HTTP body and return one scripted result."""

    def do_POST(self):
      data = self.rfile.read(int(self.headers["Content-Length"]))
      authorization.append(self.headers.get("Authorization"))
      posted.append((self.path, json.loads(data)))
      code, output = replies[len(posted) - 1]
      payload = json.dumps({"id": "resp_fixture", "status": "completed", "output": output}).encode()
      self.send_response(code)
      self.send_header("Content-Type", "application/json")
      self.send_header("Content-Length", str(len(payload)))
      self.end_headers()
      self.wfile.write(payload)

    def log_message(self, *_args):
      """Keep routine HTTP logging out of CTest output."""

  with ThreadingHTTPServer(("127.0.0.1", 0), Handler) as server:
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    config = {"dataset": {"memory_dir": str(dataset)},
              "build": {"observations_per_batch": 2, "max_tool_rounds": 1, "resume": True},
              "model": {"base_url": f"http://127.0.0.1:{server.server_port}/v1", "model": "fixture",
                        "api_key_file": key_file.name, "timeout_s": 3}}

    def run(expected, request=None):
      """Run a new process with the same SQLite work directory."""
      config_path.write_text(json.dumps(config), encoding="utf-8")
      command = [str(binary), "--config", str(config_path)]
      inputs = ""
      if request is None:
        command.append("--build")
      else:
        inputs = json.dumps(request) + "\n"
      result = subprocess.run(command, input=inputs, capture_output=True, text=True, timeout=20, check=False)
      assert result.returncode == expected, (result.stdout, result.stderr)
      return result

    def state():
      """Inspect durable state after the CLI has exited."""
      with sqlite3.connect(database) as connection:
        progress, = connection.execute("SELECT progress_json FROM build_progress").fetchone()
        receipts = dict(connection.execute("SELECT call_id,result_json FROM tool_calls"))
        entities, = connection.execute("SELECT count(*) FROM entities").fetchone()
      return json.loads(progress), receipts, entities

    try:
      run(0, {"name": "search_entities", "arguments": {}})
      missing_key = run(1)
      assert "Cannot read model key" in missing_key.stderr and key_file.name in missing_key.stderr
      assert not posted
      key_file.write_text('api_key: "fixture-key"\n', encoding="utf-8")
      with sqlite3.connect(database) as connection:
        connection.execute("CREATE TRIGGER fail_second BEFORE INSERT ON tool_calls "
                           "WHEN NEW.call_id='call_02' BEGIN SELECT RAISE(ABORT,'receipt failed'); END")
      failed = run(1)
      assert "receipt failed" in failed.stderr
      progress, receipts, entities = state()
      assert progress["next_observation"] == 0
      assert progress["active"]["pending"] == [reasoning, first, second]
      assert entities == 1 and list(receipts) == ["call_01"]
      first_receipt = receipts["call_01"]
      assert len(posted) == 1
      path, first_body = posted[0]
      assert path == "/v1/responses" and first_body["store"] is False
      assert {tool["name"] for tool in first_body["tools"]} == {"create_entity", "update_entity"}
      assert all(isinstance(tool["parameters"]["required"], list) for tool in first_body["tools"])
      original_content = first_body["input"][0]["content"]
      images = [item["image_url"] for item in original_content if item["type"] == "input_image"]
      assert len(images) == 2
      for image, obs in zip(images, observations):
        assert base64.b64decode(image.split(",", 1)[1]) == (dataset / obs["rgb_media"]).read_bytes()

      with sqlite3.connect(database) as connection:
        connection.execute("DROP TRIGGER fail_second")
      failure = run(1)
      assert "503" in failure.stderr and len(posted) == 2
      progress, receipts, entities = state()
      assert progress["next_observation"] == 0 and progress["active"]["rounds"] == 1
      assert entities == 2 and receipts["call_01"] == first_receipt
      followup = posted[1][1]["input"]
      assert followup[0]["content"] == original_content and followup[1:4] == [reasoning, first, second]
      assert followup[4] == {"type": "function_call_output", "call_id": "call_01", "output": first_receipt}
      first_entity = json.loads(first_receipt)["entity"]
      second_entity = json.loads(followup[5]["output"])["entity"]
      assert second_entity["semantic_label"] == "second"
      update = call_item("call_03", "update_entity", {
          "entity_id": first_entity["id"], "expected_revision": first_entity["revision"],
          "relations": [{"type": "on", "entity_ids": ["$self", second_entity["id"]],
                         "evidence_observation_ids": ids}]})
      replies[2] = (200, [update])

      capped = run(1)
      assert "round limit" in capped.stderr
      progress, receipts, entities = state()
      assert progress["active"]["pending"] == [update] and "call_03" not in receipts
      assert entities == 2 and progress["next_observation"] == 0

      config["build"]["max_tool_rounds"] = 2
      finished = run(0)
      assert json.loads(finished.stdout)["processed_observations"] == 2
      progress, receipts, entities = state()
      assert progress == {"next_observation": 2, "active": None} and len(receipts) == 3 and entities == 2
      assert len(posted) == 4 and posted[-1][1]["input"][-2] == update
      assert posted[-1][1]["input"][-1]["output"] == receipts["call_03"]
      assert len(json.loads(receipts["call_03"])["relations"]) == 1
      with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT predicate FROM entity_relations").fetchall() == [("on",)]

      assert authorization == ["Bearer fixture-key"] * len(posted)
      key_file.unlink()
      skipped = run(0)
      assert json.loads(skipped.stdout)["processed_observations"] == 0 and len(posted) == 4
      replay = run(0, first)
      assert json.loads(replay.stdout) == json.loads(first_receipt)
      conflict = run(1, {**first, "arguments": create("different", ids[0])})
      assert json.loads(conflict.stdout)["status"] == "call_id_conflict"
      assert state()[2] == 2
    finally:
      server.shutdown()
      thread.join()


def main():
  """Use a direct BagBuilder fixture so each tool has real media and calibration to read."""
  parser = argparse.ArgumentParser()
  parser.add_argument("--engine", type=Path, required=True)
  parser.add_argument("--fixture-pointer", type=Path, required=True)
  parser.add_argument("--output-parent", type=Path, required=True)
  args = parser.parse_args()
  fixture = Path((args.fixture_pointer / "latest.txt").read_text().strip())
  args.output_parent.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix="model-", dir=args.output_parent) as work:
    root = Path(work)
    dataset = root / "memory"
    shutil.copytree(fixture / "direct_source_png16", dataset)
    run_recovery(args.engine, dataset, root / "model.yaml")


if __name__ == "__main__":
  main()

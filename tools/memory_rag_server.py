"""Expose the local published memory search as a ROS 2 service."""

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vln_agent.srv import SearchMemory
import yaml

from memory_rag import DEFAULT_CONFIG, MemorySearch, load_config


def main():
    """Load one published version and serve requests serially in the ROS executor."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG, type=Path)
    args = parser.parse_args()
    try:
        settings = load_config(args.config)
        search = MemorySearch(settings)
    except (OSError, ValueError, KeyError, ImportError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(1, f"memory_rag_server: {error}\n")

    def reply(request, response):
        """Return JSON evidence or an error for this ROS request."""
        try:
            result = search.query(request.text, request.relation or None)
            response.result_json = json.dumps(result, ensure_ascii=False)
        except (OSError, ValueError, KeyError, ImportError, RuntimeError) as error:
            response.error = str(error)
        return response

    rclpy.init(args=[])
    try:
        node = Node("memory_rag")
        try:
            service_name = settings["service_name"]
            node.create_service(SearchMemory, service_name, reply)
            version = search.metadata["memory_version"]
            node.get_logger().info(f"Serving {version} on {service_name}")
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

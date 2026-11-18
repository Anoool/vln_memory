"""通过 ROS 2 服务查询本地已发布的空间记忆。"""

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
    """加载一个发布版本，在 ROS 执行器中串行处理请求。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG, type=Path)
    args = parser.parse_args()
    try:
        settings = load_config(args.config)
        search = MemorySearch(settings)
    except (OSError, ValueError, KeyError, ImportError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(1, f"memory_rag_server: {error}\n")

    def reply(request, response, *, vector_only=False):
        """为当前 ROS 请求返回 JSON 格式的证据或错误信息。"""
        try:
            result = search.query(request.text, request.relation or None, vector_only=vector_only)
            response.result_json = json.dumps(result, ensure_ascii=False)
        except (OSError, ValueError, KeyError, ImportError, RuntimeError) as error:
            response.error = str(error)
        return response

    def context_reply(request, response):
        """对原始用户文字执行纯向量检索，返回待核对候选。"""
        return reply(request, response, vector_only=True)

    rclpy.init(args=[])
    try:
        node = Node("memory_rag")
        try:
            service_name = settings["service_name"]
            context_service_name = settings["context_service_name"]
            node.create_service(SearchMemory, service_name, reply)
            node.create_service(SearchMemory, context_service_name, context_reply)
            version = search.metadata["memory_version"]
            node.get_logger().info(f"Serving {version} on {service_name} and {context_service_name}")
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

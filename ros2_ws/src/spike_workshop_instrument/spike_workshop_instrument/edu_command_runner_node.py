import os
import re
import signal
import subprocess
import threading
import traceback

import rclpy
from rclpy.node import Node

from spike_workshop_interfaces.srv import RunCommand


class EduCommandRunnerNode(Node):
    _TOPIC_RE = re.compile(r"^/?[A-Za-z0-9_/]+$")

    def __init__(self) -> None:
        super().__init__("edu_command_runner_node")
        self._launch_processes = {}
        self._launch_lock = threading.Lock()

        self._service = self.create_service(
            RunCommand,
            "/edu/run_command",
            self._on_run_command,
        )

        self.get_logger().info(
            "edu_command_runner_node ready. Service: /edu/run_command "
            "(allowed command IDs only)"
        )

    def destroy_node(self) -> bool:
        self._stop_all_launches()
        return super().destroy_node()

    def _on_run_command(self, request: RunCommand.Request, response: RunCommand.Response):
        command_id = str(request.command_id or "").strip()
        argument = str(request.argument or "").strip()

        if not command_id:
            response.success = False
            response.output = "command_id is required."
            return response

        try:
            success, output = self._dispatch(command_id, argument)
            response.success = bool(success)
            response.output = str(output)
            return response
        except Exception as exc:
            self.get_logger().error(
                f"Unhandled /edu/run_command error for '{command_id}': {exc}\n"
                f"{traceback.format_exc()}"
            )
            response.success = False
            response.output = f"Error while running '{command_id}': {exc}"
            return response

    def _dispatch(self, command_id: str, argument: str):
        if command_id == "list_nodes":
            return self._run_oneshot(["ros2", "node", "list"])
        if command_id == "list_topics":
            return self._run_oneshot(["ros2", "topic", "list"])
        if command_id == "list_services":
            return self._run_oneshot(["ros2", "service", "list"])
        if command_id == "launch_ui_backend":
            return self._start_launch_process(
                process_key="ui_backend",
                cli=["ros2", "launch", "spike_workshop_ui_backend", "ui_backend.launch.py"],
            )
        if command_id == "launch_instrument":
            return self._start_launch_process(
                process_key="instrument",
                cli=["ros2", "launch", "spike_workshop_instrument", "instrument.launch.py"],
            )

        if command_id.startswith("topic_info:"):
            topic_name = self._validate_topic(command_id.split(":", 1)[1] or argument)
            return self._run_oneshot(["ros2", "topic", "info", topic_name])
        if command_id.startswith("topic_hz:"):
            topic_name = self._validate_topic(command_id.split(":", 1)[1] or argument)
            return self._run_topic_hz(topic_name)

        if command_id == "topic_info":
            topic_name = self._validate_topic(argument)
            return self._run_oneshot(["ros2", "topic", "info", topic_name])
        if command_id == "topic_hz":
            topic_name = self._validate_topic(argument)
            return self._run_topic_hz(topic_name)

        return False, (
            f"Unsupported command_id '{command_id}'. Allowed: "
            "list_nodes, list_topics, list_services, launch_ui_backend, "
            "launch_instrument, topic_info:<topic>, topic_hz:<topic>."
        )

    def _validate_topic(self, topic_name: str) -> str:
        topic = str(topic_name or "").strip()
        if not topic:
            raise ValueError("Topic argument is required.")
        if not self._TOPIC_RE.fullmatch(topic):
            raise ValueError(f"Unsupported topic name '{topic}'.")
        return topic

    def _run_oneshot(self, cli, timeout_sec=None):
        self.get_logger().info(f"Executing allowed command: {' '.join(cli)}")
        try:
            completed = subprocess.run(
                cli,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
            output = self._combine_output(completed.stdout, completed.stderr)
            if not output:
                output = "(no output)"
            return completed.returncode == 0, output
        except subprocess.TimeoutExpired as exc:
            output = self._combine_output(exc.stdout, exc.stderr)
            if output:
                output += "\n"
            output += f"[edu_command_runner] Timed out after {timeout_sec:.0f} seconds."
            return False, output
        except Exception as exc:
            return False, f"Failed to execute command: {exc}"

    def _run_topic_hz(self, topic_name: str):
        success, output = self._run_oneshot(
            ["ros2", "topic", "hz", topic_name],
            timeout_sec=5.0,
        )
        # ros2 topic hz is continuous; timeout is expected. Treat timeout output as success.
        if not success and "Timed out after 5 seconds." in output:
            return True, output
        return success, output

    def _start_launch_process(self, process_key: str, cli):
        with self._launch_lock:
            if process_key == "ui_backend" and self._graph_has_any_node(
                ["/rosbridge_websocket", "/rosapi_node", "/edu_command_runner_node"]
            ):
                return True, "ui_backend appears to already be running."
            if process_key == "instrument" and self._graph_has_any_node(
                ["/instrument_node", "/spike_hw_client_node"]
            ):
                return True, "instrument appears to already be running."

            existing = self._launch_processes.get(process_key)
            if existing and existing.poll() is None:
                return True, (
                    f"{process_key} already running (pid {existing.pid})."
                )

            self._launch_processes.pop(process_key, None)

            self.get_logger().info(
                f"Starting managed launch process '{process_key}': {' '.join(cli)}"
            )
            process = subprocess.Popen(
                cli,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._launch_processes[process_key] = process
            return True, f"Started {process_key} (pid {process.pid})."

    def _graph_has_any_node(self, node_names):
        success, output = self._run_oneshot(["ros2", "node", "list"], timeout_sec=2.0)
        if not success and "Timed out" in output:
            return False
        active_nodes = set(str(line).strip() for line in output.splitlines() if str(line).strip())
        return any(node in active_nodes for node in node_names)

    @staticmethod
    def _combine_output(stdout_text, stderr_text) -> str:
        stdout_part = (stdout_text or "").strip()
        stderr_part = (stderr_text or "").strip()
        if stdout_part and stderr_part:
            return f"{stdout_part}\n{stderr_part}"
        return stdout_part or stderr_part

    def _stop_all_launches(self) -> None:
        with self._launch_lock:
            for process_key, process in list(self._launch_processes.items()):
                self._stop_process(process_key, process)
            self._launch_processes.clear()

    def _stop_process(self, process_key: str, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=2.0)
            self.get_logger().info(
                f"Stopped managed launch process '{process_key}' (pid {process.pid})."
            )
        except Exception:
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except Exception:
                process.kill()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EduCommandRunnerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

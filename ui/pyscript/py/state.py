from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Deque, List, Tuple

DEFAULT_ROSBRIDGE_URL = "ws://localhost:9090"
STATUS_HISTORY_MAX = 20
LOG_HISTORY_MAX = 20
UI_LOG_MAX = 120


HistoryEntry = Tuple[str, str]


def timestamp_now() -> str:
    return datetime.now().strftime("%H:%M:%S")


@dataclass
class UiState:
    ros_url: str = DEFAULT_ROSBRIDGE_URL
    conn_kind: str = ""
    conn_text: str = "Disconnected"
    connected: bool = False
    last_error: str = ""
    last_cmd: str = "(no command published yet)"
    status_latest: str = "(no messages yet)"
    status_history: Deque[HistoryEntry] = field(
        default_factory=lambda: deque(maxlen=STATUS_HISTORY_MAX)
    )
    topics: List[str] = field(default_factory=list)
    logs_source_text: str = "Log source: none"
    logs_history: Deque[HistoryEntry] = field(
        default_factory=lambda: deque(maxlen=LOG_HISTORY_MAX)
    )
    ui_log_lines: Deque[str] = field(default_factory=lambda: deque(["(ready)"], maxlen=UI_LOG_MAX))

    def push_status(self, text: str) -> None:
        normalized = str(text or "")
        self.status_latest = normalized or "(empty string)"
        self.status_history.appendleft((timestamp_now(), self.status_latest))

    def clear_logs(self) -> None:
        self.logs_history.clear()

    def push_log(self, text: str) -> None:
        self.logs_history.appendleft((timestamp_now(), str(text or "")))

    def push_ui_log(self, message: str) -> None:
        line = f"[{timestamp_now()}] {message}"
        if len(self.ui_log_lines) == 1 and self.ui_log_lines[0] == "(ready)":
            self.ui_log_lines.clear()
        self.ui_log_lines.appendleft(line)

    def set_topics(self, topics: List[str]) -> None:
        self.topics = sorted({str(t) for t in topics})

    def set_error(self, error_text: str) -> None:
        self.last_error = str(error_text or "").strip()


ui_state = UiState()

from __future__ import annotations

from js import document

from state import UiState


def _el(element_id: str):
    return document.getElementById(element_id)


def _set_text(element_id: str, text: str) -> None:
    el = _el(element_id)
    if el is None:
        return
    el.textContent = str(text)


def _set_input_value(element_id: str, value: str) -> None:
    el = _el(element_id)
    if el is None:
        return
    el.value = str(value)


def _clear_children(element_id: str) -> None:
    el = _el(element_id)
    if el is None:
        return
    el.innerHTML = ""


def _append_list_item(list_el, text: str, class_name: str = "") -> None:
    li = document.createElement("li")
    if class_name:
        li.className = class_name
    li.textContent = str(text)
    list_el.appendChild(li)


def _set_conn_dot(kind: str) -> None:
    dot = _el("conn-dot")
    if dot is None:
        return
    dot.classList.remove("connected", "connecting", "error")
    if kind:
        dot.classList.add(kind)


def _set_chip(element_id: str, topic_name: str, present):
    chip = _el(element_id)
    if chip is None:
        return
    chip.classList.remove("ok", "bad")
    if present is True:
        chip.classList.add("ok")
        chip.textContent = f"{topic_name}: present"
    elif present is False:
        chip.classList.add("bad")
        chip.textContent = f"{topic_name}: missing"
    else:
        chip.textContent = f"{topic_name}: unknown"


def render_connection(state: UiState) -> None:
    _set_input_value("ws-url", state.ros_url)
    _set_conn_dot(state.conn_kind)
    _set_text("conn-text", state.conn_text)


def render_error(state: UiState) -> None:
    _set_text("error-text", state.last_error or "(none)")


def render_last_command(state: UiState) -> None:
    _set_text("last-cmd", state.last_cmd)


def render_status(state: UiState) -> None:
    _set_text("status-latest", state.status_latest)
    _set_text("status-count", str(len(state.status_history)))

    list_el = _el("status-history")
    if list_el is None:
        return
    list_el.innerHTML = ""
    if not state.status_history:
        _append_list_item(list_el, "Waiting for messages...", "muted")
        return
    for stamp, text in state.status_history:
        _append_list_item(list_el, f"[{stamp}] {text}", "mono")


def render_topics(state: UiState) -> None:
    topics = list(state.topics)
    topic_set = set(topics)
    has_topics = len(topics) > 0

    _set_chip("topic-status-chip", "/status", topic_set.__contains__("/status") if has_topics else None)
    _set_chip("topic-cmd-chip", "/spike/cmd", topic_set.__contains__("/spike/cmd") if has_topics else None)

    if not topics:
        _set_text("topic-list-output", "(no topic list yet)")
        return

    lines = [f"{idx}. {name}" for idx, name in enumerate(topics, start=1)]
    _set_text("topic-list-output", "\n".join(lines))


def render_logs(state: UiState) -> None:
    _set_text("logs-source", state.logs_source_text)
    list_el = _el("logs-history")
    if list_el is None:
        return
    list_el.innerHTML = ""
    if not state.logs_history:
        _append_list_item(list_el, "No log messages yet.", "muted")
        return
    for stamp, text in state.logs_history:
        _append_list_item(list_el, f"[{stamp}] {text}", "mono")


def render_ui_log(state: UiState) -> None:
    if not state.ui_log_lines:
        _set_text("ui-log", "(ready)")
        return
    _set_text("ui-log", "\n".join(state.ui_log_lines))


def render_all(state: UiState) -> None:
    render_connection(state)
    render_error(state)
    render_last_command(state)
    render_status(state)
    render_topics(state)
    render_logs(state)
    render_ui_log(state)

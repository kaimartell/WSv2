import time
import threading
from dataclasses import dataclass
from typing import Optional

RAW_WRITE_CHUNK_BYTES = 128


@dataclass
class RawExecResult:
    ok: bool
    stdout: str = ""
    stderr: str = ""
    error: str = ""


class SerialMicroPythonClient:
    """Defensive MicroPython RAW REPL client for USB serial transport."""

    def __init__(
        self,
        *,
        port: str,
        baud: int = 115200,
        io_timeout: float = 0.1,
        prompt_timeout: float = 2.0,
        exec_timeout: float = 8.0,
        sync_retries: int = 2,
    ) -> None:
        self._port = port
        self._baud = int(baud)
        self._io_timeout = max(0.05, float(io_timeout))
        self._prompt_timeout = max(0.2, float(prompt_timeout))
        self._exec_timeout = max(0.5, float(exec_timeout))
        self._sync_retries = max(2, int(sync_retries))

        self._serial_mod = None
        self._serial_conn = None
        self._io_lock = threading.Lock()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self._serial_conn is not None:
            return

        try:
            import serial  # type: ignore
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"pyserial unavailable: {exc}") from exc

        self._serial_mod = serial
        try:
            self._serial_conn = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=self._io_timeout,
                write_timeout=1.0,
            )
            self._flush_buffers()
        except Exception as exc:  # noqa: BLE001
            self._serial_conn = None
            raise RuntimeError(f"failed opening serial port {self._port}: {exc}") from exc

    def close(self) -> None:
        if self._serial_conn is None:
            return
        try:
            self._serial_conn.close()
        except Exception:  # noqa: BLE001
            pass
        finally:
            self._serial_conn = None

    def execute_raw(
        self,
        code: str,
        timeout: Optional[float] = None,
        retry_on_syntax_error: bool = False,
    ) -> RawExecResult:
        return self.execute_raw_with_options(
            code=code,
            timeout=timeout,
            retry_on_syntax_error=retry_on_syntax_error,
        )

    def execute_raw_with_options(
        self,
        *,
        code: str,
        timeout: Optional[float] = None,
        retry_on_syntax_error: bool = False,
    ) -> RawExecResult:
        exec_timeout = self._exec_timeout if timeout is None else max(0.5, float(timeout))
        syntax_retry_budget = 1 if retry_on_syntax_error else 0

        with self._io_lock:
            try:
                self.open()
            except Exception as exc:  # noqa: BLE001
                return RawExecResult(ok=False, error=str(exc))

            if self._serial_conn is None:
                return RawExecResult(ok=False, error="serial not open")

            syntax_attempt = 0
            while syntax_attempt <= syntax_retry_budget:
                for attempt in range(1, self._sync_retries + 1):
                    entered, enter_output = self._enter_raw_repl()
                    if not entered:
                        if attempt < self._sync_retries:
                            self._recover_connection()
                            continue
                        return RawExecResult(
                            ok=False,
                            error=(
                                "raw repl sync failed; reset hub and close apps using serial (for example LEGO SPIKE app)"
                            ),
                            stderr=enter_output,
                        )

                    result = self._execute_code_raw(code=code, timeout=exec_timeout)
                    self._exit_raw_repl()
                    if (
                        retry_on_syntax_error
                        and syntax_attempt < syntax_retry_budget
                        and self._looks_like_syntax_error(result)
                    ):
                        self._recover_connection()
                        syntax_attempt += 1
                        continue
                    return result

                return RawExecResult(
                    ok=False,
                    error="raw repl sync failed; reset hub and close apps using serial (for example LEGO SPIKE app)",
                )

            return RawExecResult(ok=False, error="raw repl execution failed")

    def _recover_connection(self) -> None:
        self.close()
        time.sleep(0.1)
        self.open()

    def _flush_buffers(self) -> None:
        if self._serial_conn is None:
            return
        try:
            self._serial_conn.reset_input_buffer()
            self._serial_conn.reset_output_buffer()
        except Exception:  # noqa: BLE001
            pass

    def _write(self, payload: bytes) -> Optional[str]:
        if self._serial_conn is None:
            return "serial not open"
        try:
            total = 0
            length = len(payload)
            while total < length:
                end = min(length, total + RAW_WRITE_CHUNK_BYTES)
                written = self._serial_conn.write(payload[total:end])
                if written is None:
                    written = 0
                if written <= 0:
                    return "serial write failed: wrote 0 bytes"
                total += written
            self._serial_conn.flush()
            return None
        except Exception as exc:  # noqa: BLE001
            return f"serial write failed: {exc}"

    def _read_available(self, size: int = 512) -> bytes:
        if self._serial_conn is None:
            return b""
        try:
            return self._serial_conn.read(size)
        except Exception:
            return b""

    def _read_until(self, marker: bytes, timeout: float) -> tuple[bool, bytes]:
        deadline = time.monotonic() + max(0.1, float(timeout))
        buf = bytearray()

        while time.monotonic() < deadline:
            chunk = self._read_available(512)
            if chunk:
                buf.extend(chunk)
                if marker in buf:
                    return True, bytes(buf)
            else:
                time.sleep(0.01)

        return False, bytes(buf)

    def _read_briefly(self, timeout: float = 0.08) -> bytes:
        deadline = time.monotonic() + max(0.01, float(timeout))
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._read_available(512)
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.005)
        return bytes(buf)

    def _enter_raw_repl(self) -> tuple[bool, str]:
        if self._serial_conn is None:
            return False, "serial not open"

        self._flush_buffers()

        # Interrupt running user code first.
        write_error = self._write(b"\x03\x03")
        if write_error:
            return False, write_error
        time.sleep(0.05)

        # Enter raw REPL.
        write_error = self._write(b"\x01")
        if write_error:
            return False, write_error

        ok, data = self._read_until(marker=b">", timeout=self._prompt_timeout)
        data += self._read_briefly()
        text = data.decode("utf-8", errors="ignore")
        if not ok:
            return False, text

        # Typical raw repl banner includes "raw REPL". Some firmwares only show ">".
        if "raw REPL" in text:
            return True, text

        if text.rstrip().endswith(">") and ">>>" not in text:
            return True, text

        return False, text

    def _execute_code_raw(self, *, code: str, timeout: float) -> RawExecResult:
        if self._serial_conn is None:
            return RawExecResult(ok=False, error="serial not open")

        code_bytes = code.encode("utf-8")
        write_error = self._write(code_bytes)
        if write_error:
            return RawExecResult(ok=False, error=write_error)

        # Ctrl-D executes raw code.
        write_error = self._write(b"\x04")
        if write_error:
            return RawExecResult(ok=False, error=write_error)

        ok, raw = self._read_until_two_eot(timeout=timeout)
        if not ok:
            return RawExecResult(
                ok=False,
                error="raw repl execution timeout",
                stderr=raw.decode("utf-8", errors="ignore"),
            )

        stdout_bytes, stderr_bytes = self._split_raw_output(raw)
        stdout = stdout_bytes.decode("utf-8", errors="ignore").strip()
        stderr = stderr_bytes.decode("utf-8", errors="ignore").strip()

        return RawExecResult(ok=True, stdout=stdout, stderr=stderr)

    def _read_until_two_eot(self, timeout: float) -> tuple[bool, bytes]:
        deadline = time.monotonic() + max(0.2, float(timeout))
        buf = bytearray()
        eot_count = 0

        while time.monotonic() < deadline:
            chunk = self._read_available(512)
            if chunk:
                buf.extend(chunk)
                eot_count += chunk.count(b"\x04")
                if eot_count >= 2:
                    return True, bytes(buf)
            else:
                time.sleep(0.01)

        return False, bytes(buf)

    def _split_raw_output(self, payload: bytes) -> tuple[bytes, bytes]:
        first = payload.find(b"\x04")
        if first < 0:
            return payload, b""

        second = payload.find(b"\x04", first + 1)
        if second < 0:
            return payload[:first], payload[first + 1 :]

        stdout = payload[:first]
        stderr = payload[first + 1 : second]

        # Raw REPL typically prefixes execution output with "OK".
        if stdout.startswith(b"OK"):
            stdout = stdout[2:]

        return stdout, stderr

    def _exit_raw_repl(self) -> None:
        if self._serial_conn is None:
            return
        # Best effort return to friendly REPL.
        self._write(b"\x02")
        self._read_until(marker=b">>>", timeout=self._prompt_timeout)

    @staticmethod
    def _looks_like_syntax_error(result: RawExecResult) -> bool:
        text = " ".join(
            part
            for part in (
                result.error or "",
                result.stdout or "",
                result.stderr or "",
            )
            if part
        ).lower()
        return "syntaxerror" in text or "invalid syntax" in text

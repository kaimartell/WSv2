import unittest

from host_agent.backends.spike_usb_backend import SpikeUsbBackend


def _build_encoded_actions(count: int = 100):
    backend = SpikeUsbBackend(serial_port="auto", motor_port="A")
    encoded = []
    for idx in range(count):
        beat_t = (idx // 4) * 125
        mode = idx % 5
        if mode == 0:
            action = {"type": "run", "speed": 0.6}
        elif mode == 1:
            action = {"type": "stop", "stop_action": "coast"}
        elif mode == 2:
            action = {"type": "run", "speed": -0.7}
        elif mode == 3:
            action = {"type": "beep", "freq_hz": 440 + (idx % 8) * 20, "duration_ms": 20, "volume": 100}
        else:
            action = {"type": "stop", "stop_action": "hold"}

        encoded_action = backend._encode_score_action(action)
        if encoded_action is None:
            raise AssertionError("Failed to encode test action")
        encoded_action["t_rel_ms"] = beat_t
        encoded.append(encoded_action)
    return backend, encoded


class ScoreBatchSnippetTests(unittest.TestCase):
    def test_score_batch_snippet_is_parseable_and_normalized(self) -> None:
        backend, encoded = _build_encoded_actions(100)
        snippet = backend._build_score_batch_snippet(
            port_letter="A",
            actions=encoded,
            initial_delay_ms=25,
        )
        check = backend._compile_check_snippet(snippet, label="unittest_batch")
        self.assertTrue(check["ok"])
        self.assertTrue(snippet.endswith("\n"))
        self.assertNotIn("\r", snippet)

    def test_compile_check_returns_line_details(self) -> None:
        backend = SpikeUsbBackend(serial_port="auto", motor_port="A")
        bad_snippet = "if True print('broken')\n"
        check = backend._compile_check_snippet(bad_snippet, label="unittest_invalid")
        self.assertFalse(check["ok"])
        self.assertGreater(check["line"], 0)
        self.assertIn("offending_line", check)
        self.assertIn("excerpt", check)

    def test_score_batch_preflight_handles_large_edge_case_score(self) -> None:
        backend = SpikeUsbBackend(serial_port="auto", motor_port="A")
        events = []
        for idx in range(100):
            t_rel_ms = (idx // 3) * 90
            if idx % 4 == 0:
                action = {"type": "run", "speed": 0.5 if idx % 8 == 0 else -0.5, "port": "A"}
            elif idx % 4 == 1:
                action = {"type": "stop", "port": "A", "stop_action": "coast';oops"}
            elif idx % 4 == 2:
                action = {
                    "type": "beep",
                    "freq_hz": 55 + idx * 5,
                    "duration_ms": 10,
                    "volume": 120,
                }
            else:
                action = {"type": "stop", "port": "A", "stop_action": "brake"}
            events.append({"event_id": f"evt_{idx+1:04d}", "t_rel_ms": t_rel_ms, "action": action})

        preflight = backend._preflight_score_batch_compile(port_letter="A", events=events)
        self.assertTrue(preflight["ok"])
        self.assertEqual(preflight["event_count"], len(events))
        self.assertTrue(preflight["compile_check"]["ok"])


if __name__ == "__main__":
    unittest.main()

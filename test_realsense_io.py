"""Run: python -m unittest -v test_realsense_io (no GPIO/camera required)."""
import json
import time
import unittest

from websockets.sync.client import connect
from realsense_io import ButtonStatusClient, ButtonStatusServer


class FakeHardware:
    def __init__(self):
        self.opened = False
        self.fail = False
        self.colors = []
        self.closed = False

    def read_open(self):
        return self.opened

    def write(self, color):
        if self.fail:
            raise OSError("LED driver failed")
        self.colors.append(color)

    def close(self):
        self.colors.append("off")
        self.closed = True


def eventually(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Condition did not become true")


class StatusTests(unittest.TestCase):
    def setUp(self):
        self.hw = FakeHardware()
        self.server = ButtonStatusServer("127.0.0.1", 0, lambda: self.hw)
        self.server.start()
        self.addCleanup(self.server.close)
        eventually(lambda: self.server.snapshot()["led_commanded_color"] == "green")

    def test_nc_debounce_and_cleanup(self):
        self.hw.opened = True
        time.sleep(0.02)  # Bounce shorter than the 50 ms stable requirement.
        self.hw.opened = False
        time.sleep(0.08)
        self.assertNotIn("red", self.hw.colors)
        self.hw.opened = True
        eventually(lambda: self.server.snapshot()["led_commanded_color"] == "red")
        self.assertTrue(self.server.snapshot()["circuit_open"])
        self.hw.opened = False
        eventually(lambda: self.server.snapshot()["led_commanded_color"] == "green")
        self.server.close()
        self.assertTrue(self.hw.closed)
        self.assertEqual(self.hw.colors[-1], "off")

    def test_snapshot_change_heartbeat_and_multiple_clients(self):
        url = f"ws://127.0.0.1:{self.server.port}/ws/status"
        with connect(url, proxy=None) as first, connect(url, proxy=None) as second:
            for ws in (first, second):
                initial = json.loads(ws.recv(timeout=3))
                self.assertEqual(initial["led_commanded_color"], "green")
                self.assertIsNone(initial["led_actual_on"])
            heartbeat = json.loads(first.recv(timeout=3))
            self.assertEqual(heartbeat["type"], "heartbeat")
            self.hw.opened = True
            for ws in (first, second):
                for _ in range(10):
                    payload = json.loads(ws.recv(timeout=3))
                    if payload["led_commanded_color"] == "red":
                        break
                self.assertEqual(payload["led_commanded_color"], "red")

    def test_driver_failure_is_not_reported_as_normal(self):
        self.hw.fail = True
        self.hw.opened = True
        eventually(lambda: self.server.snapshot()["error"] is not None)
        payload = self.server.snapshot()
        self.assertFalse(payload["io_running"])
        self.assertTrue(payload["circuit_open"])
        self.assertIsNone(payload["led_actual_on"])

    def test_client_disconnect_reconnect_and_stale_snapshot(self):
        client = ButtonStatusClient("127.0.0.1", self.server.port)
        client.start()
        self.addCleanup(client.close)
        eventually(lambda: client.snapshot() is not None)
        with client.lock:
            client.received_at = time.monotonic() - 5
        self.assertIsNone(client.snapshot())
        port = self.server.port
        self.server.close()
        eventually(lambda: client.snapshot() is None)
        self.hw.opened = True
        replacement = ButtonStatusServer("127.0.0.1", port, lambda: self.hw)
        replacement.start()
        self.addCleanup(replacement.close)
        eventually(lambda: client.snapshot() is not None
                   and client.snapshot().get("led_commanded_color") == "red")


if __name__ == "__main__":
    unittest.main()

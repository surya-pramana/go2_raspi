"""Local NC/NeoPixel control and independent WebSocket status transport.

Status describes commands accepted by the LED driver, not measured light.
Snapshots aren't a durable event log; reconnect always receives current state.
"""
import json
import threading
import time
import uuid


class NeoPixelHardware:
    def __init__(self):
        # Lazy imports: the laptop never needs Raspberry Pi packages.
        import board
        import neopixel
        import RPi.GPIO as GPIO

        self.gpio = GPIO
        self.pixels = None
        GPIO.setmode(GPIO.BCM)
        try:
            GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.pixels = neopixel.NeoPixel(
                board.D18, 30, brightness=0.1, auto_write=False)
            self.write("off")
        except BaseException:
            self.close()
            raise

    def read_open(self):
        return self.gpio.input(17) == self.gpio.HIGH

    def write(self, color):
        self.pixels.fill({"off": (0, 0, 0), "red": (255, 0, 0),
                          "green": (0, 255, 0)}[color])
        self.pixels.show()

    def close(self):
        try:
            if self.pixels is not None:
                try:
                    self.write("off")
                finally:
                    self.pixels.deinit()
        finally:
            self.gpio.cleanup(17)


class ButtonStatusServer:
    def __init__(self, host="0.0.0.0", port=8765,
                 hardware_factory=NeoPixelHardware, debounce=0.05):
        self.host, self.port = host, port
        self.hardware_factory, self.debounce = hardware_factory, debounce
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.state = dict(session_id=uuid.uuid4().hex, revision=0,
                          io_running=False, circuit_open=None,
                          led_commanded_color="unknown", led_commanded_on=None,
                          led_actual_on=None, error=None)
        self.server = self.worker = self.listener = None

    def update(self, **changes):
        with self.condition:
            self.state.update(changes)
            self.state["revision"] += 1
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return dict(self.state)

    def start(self):
        from websockets.sync.server import serve
        self.server = serve(self._connection, self.host, self.port,
                            ping_interval=2, ping_timeout=3, close_timeout=1)
        self.port = self.server.socket.getsockname()[1]
        self.listener = threading.Thread(target=self.server.serve_forever,
                                         daemon=True, name="status-websocket")
        self.worker = threading.Thread(target=self._monitor,
                                       name="button-neopixel")
        self.listener.start()
        self.worker.start()
        print(f"[IO] WebSocket ws://{self.host}:{self.port}/ws/status", flush=True)

    def _monitor(self):
        hardware = None
        try:
            hardware = self.hardware_factory()
            self.update(io_running=True, led_commanded_color="off",
                        led_commanded_on=False)
            candidate = stable = None
            since = time.monotonic()
            while not self.stop_event.is_set():
                opened = hardware.read_open()
                now = time.monotonic()
                if opened != candidate:
                    candidate, since = opened, now
                elif candidate != stable and now - since >= self.debounce:
                    stable = candidate
                    color = "red" if stable else "green"
                    # Publish the input even if the LED driver subsequently fails.
                    self.update(circuit_open=stable, led_commanded_color="unknown",
                                led_commanded_on=None)
                    hardware.write(color)
                    self.update(led_commanded_color=color, led_commanded_on=True)
                    print(f"[IO] NC={'OPEN/STOP' if stable else 'CLOSED/NORMAL'} "
                          f"LED command={color}", flush=True)
                self.stop_event.wait(0.01)
        except Exception as exc:
            self.update(error=str(exc), io_running=False,
                        led_commanded_color="unknown", led_commanded_on=None)
            print(f"[IO][ERROR] {exc}", flush=True)
        finally:
            try:
                if hardware is not None:
                    hardware.close()
                    self.update(led_commanded_color="off", led_commanded_on=False)
            except Exception as exc:
                self.update(error=str(exc), led_commanded_color="unknown",
                            led_commanded_on=None)
            self.update(io_running=False)

    def _connection(self, ws):
        from websockets.exceptions import ConnectionClosed
        if ws.request.path != "/ws/status":
            ws.close(1008, "Use /ws/status")
            return
        revision = -1
        try:
            while not self.stop_event.is_set():
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.state["revision"] != revision
                        or self.stop_event.is_set(), timeout=1)
                    if self.stop_event.is_set():
                        break
                    payload = dict(self.state)
                payload["type"] = ("status" if payload["revision"] != revision
                                   else "heartbeat")
                ws.send(json.dumps(payload))
                revision = payload["revision"]
        except ConnectionClosed:
            pass

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        if self.worker is not None:
            self.worker.join()
        if self.server is not None:
            self.server.shutdown()
        if self.listener is not None:
            self.listener.join(timeout=3)


class ButtonStatusClient:
    """Background reconnect; snapshot becomes unknown when heartbeat is stale."""
    def __init__(self, host, port=8765, on_event=None):
        self.url = f"ws://{host}:{port}/ws/status"
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.payload, self.received_at = None, 0
        self.ws = None
        self.on_event = on_event
        self.thread = threading.Thread(target=self._run, daemon=True,
                                       name="button-status-client")

    def start(self):
        # Fail early with an actionable dependency error.
        from websockets.sync.client import connect  # noqa: F401
        self.thread.start()

    def snapshot(self):
        with self.lock:
            if self.payload is None or time.monotonic() - self.received_at > 4:
                return None
            return dict(self.payload)

    def _run(self):
        from websockets.sync.client import connect
        previous_camera_event = None
        previous_resource_sample = None
        while not self.stop_event.is_set():
            try:
                with connect(self.url, proxy=None, open_timeout=3,
                             close_timeout=1, ping_interval=2, ping_timeout=3,
                             max_size=16384) as ws:
                    with self.lock:
                        self.ws = ws
                    connected = False
                    while not self.stop_event.is_set():
                        payload = json.loads(ws.recv(timeout=4))
                        if not isinstance(payload, dict) or "io_running" not in payload:
                            raise ValueError("Invalid IO status")
                        with self.lock:
                            self.payload = payload
                            self.received_at = time.monotonic()
                        if self.on_event:
                            if not connected:
                                self.on_event('websocket_connected', self.url)
                            camera_event = payload.get('camera_event')
                            if camera_event and camera_event.get('id') != previous_camera_event:
                                previous_camera_event = camera_event.get('id')
                                self.on_event(camera_event.get('name', 'camera_error'),
                                              camera_event.get('detail', ''))
                            resources = payload.get('resources')
                            if resources and resources.get('resource_sample_id') != previous_resource_sample:
                                previous_resource_sample = resources.get('resource_sample_id')
                                self.on_event('pi_resources', '', **resources)
                        connected = True
            except Exception as exc:
                if not self.stop_event.is_set():
                    print(f"[IO] Status unavailable: {exc}; reconnecting", flush=True)
                    if self.on_event:
                        self.on_event('websocket_unavailable', str(exc))
            finally:
                with self.lock:
                    self.payload = None
                    self.ws = None
            self.stop_event.wait(1)

    def close(self):
        self.stop_event.set()
        with self.lock:
            ws = self.ws
        if ws is not None:
            ws.close()
        if self.thread.is_alive():
            self.thread.join(timeout=5)

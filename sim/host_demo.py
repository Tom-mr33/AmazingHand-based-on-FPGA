"""Interactive Host -> MuJoCo loop. Inputs arrive live, never from a timeline."""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))
from app.control.backend_mujoco import MujocoBackend
from app.strategy.grasp_selector import GraspSelector

ACTIONS = ("open", "grasp", "hold", "release", "stop", "reset")


class LiveController:
    """Manual Host input uses the existing selector and backend grasp FSM."""
    def __init__(self, backend=None):
        self.backend = backend or MujocoBackend()
        self.selector = GraspSelector()
        self.category = "banana"
        self.events = deque(maxlen=60)
        self.message = "Ready. Select an object, then issue a command."
        self._seq = 0

    def command(self, action, category=None):
        if action not in ACTIONS:
            raise ValueError("unknown action")
        if category is not None:
            if category not in self.selector.known_objects:
                raise ValueError("choose banana, apple or bottle")
            self.category = category
        self._seq += 1
        try:
            if action == "grasp":
                plan = self.selector.select(self.category)
                # Model zero is its open pose. Mechanical servo pregrasp offsets
                # in the uncalibrated table must not be presented as model zero.
                plan = replace(plan, pregrasp_deg=(0.0,) * 8)
                self.backend.start_grasp_fsm(plan)
            elif action == "open":
                self.backend.open_hand()
            else:
                getattr(self.backend, action if action != "stop" else "emergency_stop")()
            self.message = f"{action}: {self.category}"
            accepted = True
        except RuntimeError as exc:
            self.message = str(exc)
            accepted = False
        self.events.append({"seq": self._seq, "action": action,
                            "category": self.category, "accepted": accepted})
        return accepted

    def tick(self, dt_s):
        self.backend.tick(dt_s)

    def telemetry(self):
        backend = self.backend
        plant = backend.plant
        snap = backend.poll_fsm()
        return {
            "source": "mujoco", "seq": self._seq, "state": snap.state.name,
            "stopped": backend.stopped, "timeout": snap.timeout,
            "fault": snap.fault, "err_bitmap": snap.err_bitmap,
            "torque_enabled": any(backend.bus.state(i).torque_on for i in range(1, 9)),
            "category": self.category, "sim_time_s": float(plant.data.time),
            "positions_deg": np.degrees(plant.positions_rad).tolist(),
            "ctrl_deg": np.degrees(plant.data.ctrl).tolist(),
            "target_raw": [backend.bus.state(i).goal_raw for i in range(1, 9)],
            "load_raw": [f.load_raw for f in snap.feedback],
            "actuator_force": plant.data.actuator_force.tolist(),
            "message": self.message, "events": list(self.events),
        }


class CommandMailbox:
    def __init__(self):
        self.queue = deque()
        self.lock = threading.Lock()
        self.state = {"state": "STARTING"}

    def submit(self, action, category=None):
        if action not in ACTIONS or (category is not None and category not in ("banana", "apple", "bottle")):
            raise ValueError("invalid action or category")
        with self.lock:
            if action == "stop":
                self.queue.clear()
                self.queue.appendleft((action, category))
            elif len(self.queue) < 32:
                self.queue.append((action, category))
            else:
                raise ValueError("command queue full")

    def drain(self):
        with self.lock:
            commands = list(self.queue)
            self.queue.clear()
            return commands

    def publish(self, state):
        with self.lock:
            self.state = state


def serve_commands(mailbox, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def send_json(self, status, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path != "/state":
                self.send_json(404, {"error": "use GET /state or POST /command"})
                return
            with mailbox.lock:
                state = mailbox.state
            self.send_json(200, state)

        def do_POST(self):
            try:
                if self.path != "/command" or self.headers.get("Content-Type") != "application/json":
                    raise ValueError("POST /command with application/json required")
                origin = self.headers.get("Origin")
                if origin and origin not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}"):
                    raise ValueError("foreign origin")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024:
                    raise ValueError("body must be 1..1024 bytes")
                self.connection.settimeout(1.0)
                value = json.loads(self.rfile.read(length))
                mailbox.submit(value["action"], value.get("category"))
                self.send_json(202, {"queued": True})
            except (ValueError, KeyError, TypeError, TimeoutError) as exc:
                self.send_json(400, {"error": str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def draw_panel(renderer, camera, controller, state):
    renderer.update_scene(controller.backend.plant.data, camera=camera)
    frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
    image = np.full((860, 1280, 3), (27, 23, 18), dtype=np.uint8)
    image[:520] = frame

    def text(value, x, y, color=(228, 232, 237), scale=0.57):
        cv2.putText(image, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    text("AMAZINGHAND  /  LIVE HOST CONTROL", 24, 34, scale=0.8)
    text(f"Object: {controller.category.upper()}  |  1 Banana   2 Apple   3 Bottle", 24, 556)
    text(f"FSM: {state['state']}   SIM: {state['sim_time_s']:.2f}s   Command #{state['seq']}", 24, 588)
    color = (70, 100, 255) if state["stopped"] else (100, 230, 150)
    text("STOP LATCHED - press E to reset" if state["stopped"] else state["message"], 24, 620, color)
    for i, action in enumerate(ACTIONS):
        x = 24 + i * 207
        cv2.rectangle(image, (x, 638), (x + 190, 682), (65, 59, 48), -1)
        text(f"{('O','G','H','R','S','E')[i]}  {action.upper()}", x + 15, 667)
    text("JOINT         1        2        3        4        5        6        7        8", 24, 718)
    for y, label, values in ((750, "Measured deg", state["positions_deg"]),
                              (779, "Control deg", state["ctrl_deg"]),
                              (808, "Load (SIM)", state["load_raw"])):
        text(label, 24, y)
        for i, value in enumerate(values):
            text(f"{value:7.1f}", 180 + i * 131, y)
    text("Mouse drag: orbit | Wheel: zoom | Q / Esc: exit | Feedback is measured by MuJoCo; loads uncalibrated.",
         24, 843, (156, 170, 186), scale=0.48)
    return image


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run real physics and local API without a window")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--duration", type=float, default=None, help="optional wall-clock duration in seconds")
    parser.add_argument("--snapshot", type=Path, help="save the last rendered panel (GUI mode)")
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("duration must be positive and finite")
    controller = LiveController()
    mailbox = CommandMailbox()
    server = None
    renderer = None
    title = "AmazingHand - Host to MuJoCo LIVE"
    done = threading.Event()
    previous = {}
    frame = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: done.set())
        controller.backend.open()
        server = serve_commands(mailbox, args.port)
        if not args.headless:
            model = controller.backend.plant.model
            model.vis.global_.offwidth = 1280
            model.vis.global_.offheight = 520
            renderer = mujoco.Renderer(model, height=520, width=1280)
            camera = mujoco.MjvCamera()
            camera.azimuth, camera.elevation, camera.distance = 130, -18, 0.43
            camera.lookat[:] = (0.02, 0, 0.14)
            drag = {"last": None}

            def mouse(event, x, y, flags, _):
                if event == cv2.EVENT_LBUTTONDOWN:
                    if 638 <= y <= 682 and 24 <= x < 1266:
                        i = (x - 24) // 207
                        if i < 6:
                            mailbox.submit(ACTIONS[i])
                    elif y < 520:
                        drag["last"] = (x, y)
                elif event == cv2.EVENT_LBUTTONUP:
                    drag["last"] = None
                elif event == cv2.EVENT_MOUSEMOVE and drag["last"]:
                    lx, ly = drag["last"]
                    camera.azimuth -= (x - lx) * 0.4
                    camera.elevation = max(-89, min(0, camera.elevation + (y - ly) * 0.3))
                    drag["last"] = (x, y)
                elif event == cv2.EVENT_MOUSEWHEEL:
                    camera.distance = max(0.25, min(2.5, camera.distance * (0.9 if flags > 0 else 1.1)))

            cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(title, mouse)
        print(f"LIVE ready: http://127.0.0.1:{args.port}/state", flush=True)
        start = last = last_draw = time.monotonic()
        while not done.is_set():
            now = time.monotonic()
            elapsed, last = now - last, now
            if elapsed > 0.5:
                controller.command("stop")
                controller.message = "Host loop overdue (>500 ms); reset required"
                mailbox.drain()
            for action, category in mailbox.drain():
                controller.command(action, category)
            controller.tick(min(elapsed, 0.05))
            state = controller.telemetry()
            mailbox.publish(state)
            if renderer and now - last_draw >= 1 / 30:
                frame = draw_panel(renderer, camera, controller, state)
                cv2.imshow(title, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key in (ord("1"), ord("2"), ord("3")):
                    controller.category = ("banana", "apple", "bottle")[key - ord("1")]
                actions = dict(zip(map(ord, "oghrse"), ACTIONS))
                if key in actions:
                    mailbox.submit(actions[key])
                last_draw = now
            if args.duration is not None and now - start >= args.duration:
                break
            time.sleep(0.002)
        return 0
    finally:
        controller.backend.close()
        if server:
            server.shutdown()
            server.server_close()
        if renderer:
            if args.snapshot and frame is not None:
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(args.snapshot), frame)
            renderer.close()
            cv2.destroyAllWindows()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())

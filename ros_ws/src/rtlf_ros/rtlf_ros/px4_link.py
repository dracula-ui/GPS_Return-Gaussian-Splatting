"""MAVSDK-Python link to PX4 running in its own thread/event loop, so it can live inside a ROS 2 node.

Provides latest NED position/attitude (with a short history buffer) and velocity-offboard control.
Requires `pip install "mavsdk<4"` (the gRPC wrapper API used here).
"""
import asyncio
import collections
import threading
import time

import numpy as np


class Px4Link:
    def __init__(self, address="udpin://0.0.0.0:14540", clock=time.monotonic, log=print):
        self.address, self.clock, self.log = address, clock, log
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.lock = threading.Lock()
        self.buf = collections.deque(maxlen=2000)       # (t, n, e, d, qw, qx, qy, qz)
        self._q = (1.0, 0.0, 0.0, 0.0)
        self.ready = False
        self.armed = False
        self.offboard_ok = False
        self.error = None
        self.drone = None

    # ---- lifecycle
    def start(self):
        self.thread.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._main())
        except Exception as ex:                          # pragma: no cover
            self.error = repr(ex); self.log("PX4 link error: %s" % ex)

    async def _main(self):
        from mavsdk import System
        d = System(); self.drone = d
        await d.connect(system_address=self.address)
        self.log("waiting for PX4 connection ...")
        async for st in d.core.connection_state():
            if st.is_connected:
                break
        self.log("waiting for PX4 position / home / estimator health ...")
        async def _wait_health():
            async for h in d.telemetry.health():
                self.log("PX4 health: global_position_ok=%s home_position_ok=%s local_position_ok=%s"
                         % (h.is_global_position_ok, h.is_home_position_ok, h.is_local_position_ok))
                if h.is_global_position_ok and h.is_home_position_ok:
                    return
        try:
            await asyncio.wait_for(_wait_health(), timeout=45.0)
        except asyncio.TimeoutError:
            self.log("home position not reported after 45 s (the mission does not use it); continuing")
        for fn, rate in (("set_rate_position_velocity_ned", 30), ("set_rate_attitude_quaternion", 30)):
            try:
                await getattr(d.telemetry, fn)(rate)
            except Exception as ex:
                self.log("could not set %s: %s" % (fn, ex))
        asyncio.ensure_future(self._att())
        asyncio.ensure_future(self._armed())
        self.ready = True
        self.log("PX4 ready")
        async for pv in d.telemetry.position_velocity_ned():
            p = pv.position
            with self.lock:
                self.buf.append((self.clock(), p.north_m, p.east_m, p.down_m) + self._q)

    async def _att(self):
        async for q in self.drone.telemetry.attitude_quaternion():
            with self.lock:
                self._q = (q.w, q.x, q.y, q.z)

    async def _armed(self):
        async for a in self.drone.telemetry.armed():
            self.armed = a

    def _call(self, coro, timeout=30.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    # ---- state
    def latest(self):
        with self.lock:
            return self.buf[-1] if self.buf else None

    def truth_at(self, t):
        """Linear interpolation of position at time t (nearest attitude). Returns (n, e, d, q) or None."""
        with self.lock:
            b = list(self.buf)
        if not b:
            return None
        ts = np.array([x[0] for x in b])
        if t <= ts[0]:
            r = b[0]
        elif t >= ts[-1]:
            r = b[-1]
        else:
            i = int(np.searchsorted(ts, t))
            a, c = b[i - 1], b[i]
            w = (t - a[0]) / max(c[0] - a[0], 1e-9)
            r = tuple(a[k] + w * (c[k] - a[k]) for k in range(len(a)))
            r = (t,) + r[1:4] + b[i][4:8]
        return r[1], r[2], r[3], r[4:8]

    # ---- control
    def arm_and_offboard(self):
        from mavsdk.offboard import VelocityNedYaw

        async def go():
            try:
                await self.drone.action.arm()
            except Exception as ex:
                self.log("arm refused (%s); forcing arm (simulation only)" % str(ex).splitlines()[0][:120])
                await self.drone.action.arm_force()
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await self.drone.offboard.start()
        self._call(go())
        self.offboard_ok = True

    def send_velocity(self, vn, ve, vd, yaw_deg=0.0):
        from mavsdk.offboard import VelocityNedYaw
        asyncio.run_coroutine_threadsafe(
            self.drone.offboard.set_velocity_ned(VelocityNedYaw(float(vn), float(ve), float(vd), float(yaw_deg))), self.loop)

    def land_and_wait(self, timeout=120.0):
        async def go():
            try:
                await self.drone.offboard.stop()
            except Exception:
                pass
            await self.drone.action.land()
            async def wait():
                async for a in self.drone.telemetry.armed():
                    if not a:
                        return
            try:
                await asyncio.wait_for(wait(), timeout)
            except asyncio.TimeoutError:
                self.log("disarm wait timed out")
        self._call(go(), timeout + 10)

"""sim3d/camera.py — Three camera modes: follow / FPV / orbit."""
import math
from panda3d.core import Vec3, NodePath

FOLLOW_OFFSET = Vec3(0, -12, 5)   # behind and above
FPV_OFFSET    = Vec3(0,  1.2, 0.2)

class CameraController:
    def __init__(self, base_app, drone_root):
        self._base  = base_app
        self._drone = drone_root
        self._mode  = 0          # 0=follow, 1=FPV, 2=orbit
        self._orbit_h   = 180.0  # horizontal angle
        self._orbit_v   = 30.0   # vertical angle
        self._orbit_dist= 15.0

        base_app.disableMouse()  # we control the camera

        # Mouse state for orbit mode
        self._mouse_last = None
        base_app.accept("mouse1",       self._mouse_down)
        base_app.accept("mouse1-up",    self._mouse_up)
        base_app.accept("wheel_up",     self._zoom_in)
        base_app.accept("wheel_down",   self._zoom_out)

    def cycle_mode(self):
        self._mode = (self._mode + 1) % 3
        names = ["FOLLOW", "FPV", "ORBIT"]
        print(f"[Camera] Mode → {names[self._mode]}")

    def update(self, dt):
        dp = self._drone.getPos()
        dh = self._drone.getH()   # yaw

        if self._mode == 0:     # ── Third-person follow ──
            # Rotate offset by drone yaw
            yaw_r = math.radians(dh)
            ox = FOLLOW_OFFSET.y * math.sin(yaw_r)
            oy = FOLLOW_OFFSET.y * math.cos(yaw_r)
            oz = FOLLOW_OFFSET.z
            target_pos = dp + Vec3(ox, oy, oz)
            # Smooth lerp
            cam_pos = self._base.camera.getPos()
            alpha = 0.08
            new_pos = cam_pos + (target_pos - cam_pos) * alpha
            self._base.camera.setPos(new_pos)
            self._base.camera.lookAt(dp + Vec3(0,0,0.5))

        elif self._mode == 1:   # ── FPV ──
            fwd_r = math.radians(dh)
            fx = dp.x + FPV_OFFSET.y * math.sin(fwd_r)
            fy = dp.y + FPV_OFFSET.y * math.cos(fwd_r)
            fz = dp.z + FPV_OFFSET.z
            self._base.camera.setPos(fx, fy, fz)
            self._base.camera.setHpr(dh, self._drone.getP(), 0)

        else:                   # ── Free orbit ──
            hr = math.radians(self._orbit_h)
            vr = math.radians(self._orbit_v)
            cx = dp.x + self._orbit_dist * math.sin(hr) * math.cos(vr)
            cy = dp.y + self._orbit_dist * math.cos(hr) * math.cos(vr)
            cz = dp.z + self._orbit_dist * math.sin(vr)
            self._base.camera.setPos(cx, cy, cz)
            self._base.camera.lookAt(dp + Vec3(0,0,0.5))

        # Mouse drag in orbit mode
        if self._mode == 2 and self._base.mouseWatcherNode.hasMouse():
            mx = self._base.mouseWatcherNode.getMouseX()
            my = self._base.mouseWatcherNode.getMouseY()
            if self._mouse_last:
                dx = mx - self._mouse_last[0]
                dy = my - self._mouse_last[1]
                self._orbit_h -= dx * 80
                self._orbit_v  = max(5, min(85, self._orbit_v + dy * 50))
            if getattr(self,'_mouse_held',False):
                self._mouse_last = (mx, my)

    def _mouse_down(self): self._mouse_held=True;  self._mouse_last=None
    def _mouse_up(self):   self._mouse_held=False; self._mouse_last=None
    def _zoom_in(self):    self._orbit_dist = max(4, self._orbit_dist-2)
    def _zoom_out(self):   self._orbit_dist = min(80, self._orbit_dist+2)

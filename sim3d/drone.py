"""sim3d/drone.py — Procedural quadcopter model + 6-DOF update."""
import math, os
from panda3d.core import NodePath, Vec3, Vec4, CollisionNode, CollisionSphere
from sim3d.geometry_utils import make_box, make_disk, make_cylinder, make_line_quad

ARM_L   = 1.8    # arm length from center to rotor (metres)
ARM_ANG = 45.0   # arm angle (X config quad)
ROTOR_R = 0.55   # rotor disk radius

class DroneModel:
    def __init__(self, base_app):
        self._base = base_app
        self.root  = base_app.render.attachNewNode("drone_root")
        self._build()
        self._rotor_angle = 0.0

    def _build(self):
        r = self.root

        # ── Body ─────────────────────────────────────────────────────────────
        body = make_box("body", 0.55, 0.55, 0.18, (0.12,0.12,0.14,1))
        body.reparentTo(r)
        # Battery bump on top
        batt = make_box("battery", 0.28, 0.18, 0.10, (0.05,0.05,0.05,1))
        batt.reparentTo(r); batt.setZ(0.14)
        # Status LED (small coloured box)
        led = make_box("led", 0.06,0.06,0.04, (0.0,0.8,1.0,1))
        led.reparentTo(r); led.setPos(0.0, 0.28, 0.10)

        # ── Arms + Rotor assemblies ──────────────────────────────────────────
        self._rotor_nodes = []
        self._blade_nodes = []
        # 4 arms at ±45° / ±135°
        arm_configs = [
            ( 45,  (0.9,0.1,0.1,1)),   # Front-Right: red
            (-45,  (0.9,0.1,0.1,1)),   # Front-Left : red
            (135,  (0.1,0.9,0.1,1)),   # Rear-Right : green
            (-135, (0.1,0.9,0.1,1)),   # Rear-Left  : green
        ]
        for h_angle, rotor_color in arm_configs:
            h_rad = math.radians(h_angle)
            ex = math.sin(h_rad)*ARM_L
            ey = math.cos(h_rad)*ARM_L

            # Arm (thin box oriented along its length)
            arm = make_box("arm", ARM_L*2, 0.18, 0.10, (0.22,0.22,0.25,1))
            arm.reparentTo(r); arm.setH(h_angle)

            # Motor mount cylinder at arm tip
            mount = make_cylinder("mount", 0.18, 0.22, 8, (0.3,0.3,0.32,1))
            mount.reparentTo(r); mount.setPos(ex, ey, 0.04)

            # Rotor disk (semi-transparent spinning disk)
            disk = make_disk("rotor_disk", ROTOR_R, 24, (*rotor_color[:3], 0.35))
            disk.reparentTo(r); disk.setPos(ex, ey, 0.14)
            disk.setTransparency(1)

            # Rotor hub (rotation pivot)
            hub = r.attachNewNode(f"rotor_hub_{h_angle}")
            hub.setPos(ex, ey, 0.16)
            self._rotor_nodes.append(hub)

            # Two blades per rotor
            for b in range(2):
                blade = make_line_quad("blade", ROTOR_R*1.8, 0.14, (*rotor_color[:3],0.85))
                blade.reparentTo(hub); blade.setH(b*90)
            self._blade_nodes.append(hub)

        # ── Landing gear ─────────────────────────────────────────────────────
        for sx, sy in [(0.5,0.5),(-0.5,0.5),(0.5,-0.5),(-0.5,-0.5)]:
            leg = make_cylinder("leg", 0.04, 0.45, 6, (0.25,0.25,0.28,1))
            leg.reparentTo(r); leg.setPos(sx*ARM_L*0.6, sy*ARM_L*0.6, -0.30)
            foot = make_box("foot", 0.25,0.06,0.04, (0.2,0.2,0.22,1))
            foot.reparentTo(r); foot.setPos(sx*ARM_L*0.6, sy*ARM_L*0.6, -0.52)

        # ── Collision sphere ──────────────────────────────────────────────────
        cs = CollisionSphere(0, 0, 0, ARM_L)
        cn = CollisionNode("drone_collision"); cn.addSolid(cs)
        self._coll_np = r.attachNewNode(cn)

        # Start at ground level
        self.root.setPos(0, 0, 0.55)

    def update(self, x, y, z, roll_deg, pitch_deg, yaw_deg, thrust_ratio, sim_time):
        """Move and orient the drone, spin rotors."""
        self.root.setPos(x, y, z + 0.55)   # +0.55 = landing gear clearance
        self.root.setHpr(yaw_deg, pitch_deg, roll_deg)

        # Spin rotors — speed ∝ thrust_ratio, cap for perf
        rpm = 120 + thrust_ratio * 480   # 0→120 rpm, full→600 rpm
        dangle = rpm * (1/60) * 360 * (1/30)   # assume ~30fps
        self._rotor_angle = (self._rotor_angle + dangle) % 360
        for hub in self._rotor_nodes:
            hub.setH(self._rotor_angle)

    def get_pos(self):
        return self.root.getPos()

    def get_nose_pos(self):
        """Position 1m in front of the drone body (FPV mount point)."""
        return self.root.getPos() + self.root.getQuat().xform(Vec3(0, 1.2, 0.2))

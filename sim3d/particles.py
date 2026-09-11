"""sim3d/particles.py — Lightweight custom VFX particle pool."""
import random, math
from panda3d.core import NodePath, CardMaker, Vec3, Vec4

MAX_PARTICLES = 50   # hard cap for low-end hardware

class Particle:
    __slots__ = ('np','vx','vy','vz','life','max_life','active')
    def __init__(self):
        self.np = None; self.active = False

class VFXSystem:
    def __init__(self, base_app, drone_root):
        self._base  = base_app
        self._drone = drone_root
        self._root  = base_app.render.attachNewNode("vfx_root")
        self._pool  = []
        self._crash_done = False
        self._dust_timer = 0.0
        self._smoke_timer = 0.0

        # Pre-build particle pool (CardMaker quads)
        cm = CardMaker("particle")
        cm.setFrame(-0.15, 0.15, -0.15, 0.15)
        for _ in range(MAX_PARTICLES):
            p = Particle()
            p.np = self._root.attachNewNode(cm.generate())
            p.np.setBillboardPointEye()
            p.np.setTransparency(1)
            p.np.hide()
            self._pool.append(p)

    def _emit(self, pos, vx, vy, vz, life, color, scale=1.0):
        for p in self._pool:
            if not p.active:
                p.np.setPos(pos); p.np.setScale(scale)
                p.np.setColor(*color)
                p.np.show()
                p.vx=vx+random.uniform(-0.3,0.3)
                p.vy=vy+random.uniform(-0.3,0.3)
                p.vz=vz+random.uniform(-0.1,0.3)
                p.life=life; p.max_life=life; p.active=True
                return

    def update(self, thrust_ratio, altitude, sim_time):
        dt = 0.033   # assume ~30fps for particle update

        # ── Advance existing particles ──────────────────────────────────────
        for p in self._pool:
            if not p.active: continue
            p.life -= dt
            if p.life <= 0:
                p.np.hide(); p.active=False; continue
            p.np.setX(p.np.getX() + p.vx*dt)
            p.np.setY(p.np.getY() + p.vy*dt)
            p.np.setZ(p.np.getZ() + p.vz*dt)
            alpha = p.life/p.max_life
            col = p.np.getColor()
            p.np.setAlphaScale(alpha)
            p.np.setScale(p.np.getScale().x * (1.0 + dt*0.8))  # grow

        # ── Downwash dust (only near ground) ────────────────────────────────
        self._dust_timer += dt
        if altitude < 5.0 and thrust_ratio > 0.1 and self._dust_timer > 0.08:
            self._dust_timer = 0.0
            dpos = self._drone.getPos()
            dpos.z = 0.1
            for _ in range(3):
                r = random.uniform(0, 1.5)
                a = random.uniform(0, 2*math.pi)
                self._emit(dpos + Vec3(r*math.cos(a), r*math.sin(a), 0),
                           random.uniform(-0.4,0.4), random.uniform(-0.4,0.4),
                           random.uniform(0.2,0.6), life=1.5,
                           color=(0.55,0.48,0.35, 0.7), scale=0.3)

        # ── Smoke trail ──────────────────────────────────────────────────────
        self._smoke_timer += dt
        if thrust_ratio > 0.05 and self._smoke_timer > 0.12:
            self._smoke_timer = 0.0
            spos = self._drone.getPos() + Vec3(0,0,0.2)
            self._emit(spos, 0, 0, 0.3, life=2.0,
                       color=(0.6,0.6,0.6,0.4), scale=0.2)

    def trigger_crash(self):
        if self._crash_done: return
        self._crash_done = True
        pos = self._drone.getPos()
        for _ in range(25):   # burst
            self._emit(pos,
                       random.uniform(-3,3), random.uniform(-3,3),
                       random.uniform(1,5), life=2.5,
                       color=(0.95,0.3+random.random()*0.3,0.0,0.9),
                       scale=0.5)
        for _ in range(10):   # debris
            self._emit(pos,
                       random.uniform(-2,2), random.uniform(-2,2),
                       random.uniform(0.5,3), life=3.0,
                       color=(0.1,0.1,0.1,0.85), scale=0.25)
        print("[VFX] CRASH EXPLOSION triggered!")

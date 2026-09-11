"""sim3d/environment.py — Terrain, sky, lighting, trees."""
import math, random
from panda3d.core import (DirectionalLight, AmbientLight, Fog,
                           CardMaker, Vec4, Vec3, NodePath,
                           GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                           GeomLines, Geom, GeomNode)
from sim3d.geometry_utils import make_box

GROUND_SIZE  = 600.0   # metres
GRID_SPACING = 20.0
NUM_TREES    = 60

def setup_environment(base_app):
    """Build and return the environment NodePath."""
    root = base_app.render.attachNewNode("environment")

    # ── Background / Sky colour ──────────────────────────────────────────────
    base_app.setBackgroundColor(0.45, 0.72, 0.95, 1.0)

    # ── Fog (depth cue, hides terrain edge) ─────────────────────────────────
    fog = Fog("scene_fog")
    fog.setColor(0.45, 0.72, 0.95)
    fog.setExpDensity(0.003)
    base_app.render.setFog(fog)

    # ── Ground plane ─────────────────────────────────────────────────────────
    cm = CardMaker("ground")
    cm.setFrame(-GROUND_SIZE/2, GROUND_SIZE/2, -GROUND_SIZE/2, GROUND_SIZE/2)
    ground = root.attachNewNode(cm.generate())
    ground.setP(-90)           # rotate to lie flat (CardMaker is in XY)
    ground.setColor(0.22, 0.45, 0.18, 1.0)

    # ── Grid lines on ground ─────────────────────────────────────────────────
    grid_node = GeomNode("grid")
    vdata = GeomVertexData("grid", GeomVertexFormat.getV3c4(), Geom.UHStatic)
    v = GeomVertexWriter(vdata, "vertex")
    c = GeomVertexWriter(vdata, "color")
    lines = GeomLines(Geom.UHStatic)
    line_color = (0.28, 0.52, 0.22, 0.5)
    half = GROUND_SIZE/2
    idx = 0
    steps = int(GROUND_SIZE / GRID_SPACING) + 1
    for i in range(steps):
        x = -half + i * GRID_SPACING
        for (x0,y0,z0),(x1,y1,z1) in [
            ((x,-half,0.05),(x,half,0.05)),
            ((-half,x,0.05),(half,x,0.05)),
        ]:
            v.addData3(x0,y0,z0); c.addData4(*line_color)
            v.addData3(x1,y1,z1); c.addData4(*line_color)
            lines.addVertices(idx, idx+1); idx += 2
    grid_geom = Geom(vdata); grid_geom.addPrimitive(lines)
    grid_node.addGeom(grid_geom)
    grid_np = root.attachNewNode(grid_node)
    grid_np.setRenderModeThickness(1.0)

    # ── Trees (billboard sprites = flat quads, always face camera) ───────────
    random.seed(42)
    tree_root = root.attachNewNode("trees")
    for _ in range(NUM_TREES):
        tx = random.uniform(-GROUND_SIZE/2+10, GROUND_SIZE/2-10)
        ty = random.uniform(-GROUND_SIZE/2+10, GROUND_SIZE/2-10)
        # Skip area near spawn
        if abs(tx) < 15 and abs(ty) < 15: continue
        th = random.uniform(3.0, 8.0)
        # Trunk
        trunk = make_box("trunk", 0.4, 0.4, th*0.4, (0.35,0.22,0.1,1))
        trunk.reparentTo(tree_root)
        trunk.setPos(tx, ty, th*0.2)
        # Canopy (billboard using 2 crossed flat quads)
        cw = random.uniform(3.0, 5.0)
        for rot in [0, 90]:
            cm2 = CardMaker("canopy")
            cm2.setFrame(-cw/2, cw/2, 0, cw)
            canopy = tree_root.attachNewNode(cm2.generate())
            canopy.setPos(tx, ty, th*0.35)
            canopy.setH(rot)
            canopy.setBillboardAxis()   # always face camera (billboard)
            canopy.setColor(
                random.uniform(0.1,0.2),
                random.uniform(0.4,0.65),
                random.uniform(0.08,0.18), 1.0)
            canopy.setTransparency(1)

    # ── Lights ───────────────────────────────────────────────────────────────
    # Sun (directional)
    sun = DirectionalLight("sun")
    sun.setColor(Vec4(1.0, 0.95, 0.85, 1.0))
    sun_np = base_app.render.attachNewNode(sun)
    sun_np.setHpr(-60, -45, 0)
    base_app.render.setLight(sun_np)

    # Ambient fill
    amb = AmbientLight("ambient")
    amb.setColor(Vec4(0.35, 0.38, 0.45, 1.0))
    amb_np = base_app.render.attachNewNode(amb)
    base_app.render.setLight(amb_np)

    return root

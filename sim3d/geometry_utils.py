"""Lightweight procedural geometry for Panda3D — no external models needed."""
import math
from panda3d.core import (GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                          GeomTriangles, Geom, GeomNode, NodePath,
                          Vec4, Vec3)

def _base_geom(name):
    fmt   = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData(name, fmt, Geom.UHStatic)
    v  = GeomVertexWriter(vdata, 'vertex')
    n  = GeomVertexWriter(vdata, 'normal')
    c  = GeomVertexWriter(vdata, 'color')
    return vdata, v, n, c

def make_box(name, w, d, h, color=(0.2,0.2,0.2,1)):
    """Axis-aligned box centered at origin."""
    vdata, v, n, c = _base_geom(name)
    hw,hd,hh = w/2, d/2, h/2
    faces = [
        # (normal, 4 vertices)
        (( 0, 0,-1),[(-hw,-hd,-hh),(hw,-hd,-hh),(hw,hd,-hh),(-hw,hd,-hh)]),
        (( 0, 0, 1),[(-hw,-hd, hh),(hw,-hd, hh),(hw,hd, hh),(-hw,hd, hh)]),
        ((-1, 0, 0),[(-hw,-hd,-hh),(-hw,hd,-hh),(-hw,hd, hh),(-hw,-hd, hh)]),
        (( 1, 0, 0),[(hw,-hd,-hh),(hw,hd,-hh),(hw,hd, hh),(hw,-hd, hh)]),
        (( 0,-1, 0),[(-hw,-hd,-hh),(hw,-hd,-hh),(hw,-hd, hh),(-hw,-hd, hh)]),
        (( 0, 1, 0),[(-hw, hd,-hh),(hw, hd,-hh),(hw, hd, hh),(-hw, hd, hh)]),
    ]
    tris = GeomTriangles(Geom.UHStatic)
    idx = 0
    for (nx,ny,nz), verts in faces:
        for x,y,z in verts:
            v.addData3(x,y,z); n.addData3(nx,ny,nz); c.addData4(*color)
        tris.addVertices(idx,idx+1,idx+2)
        tris.addVertices(idx,idx+2,idx+3)
        idx += 4
    geom = Geom(vdata); geom.addPrimitive(tris)
    node = GeomNode(name); node.addGeom(geom)
    return NodePath(node)

def make_disk(name, radius, segs=20, color=(0.1,0.8,0.1,0.6)):
    """Flat disk in XY plane centered at origin."""
    vdata, v, n, c = _base_geom(name)
    tris = GeomTriangles(Geom.UHStatic)
    # Center vertex
    v.addData3(0,0,0); n.addData3(0,0,1); c.addData4(*color)
    for i in range(segs):
        a = 2*math.pi*i/segs
        v.addData3(radius*math.cos(a), radius*math.sin(a), 0)
        n.addData3(0,0,1); c.addData4(*color)
    for i in range(segs):
        tris.addVertices(0, 1+i, 1+(i+1)%segs)
    geom = Geom(vdata); geom.addPrimitive(tris)
    node = GeomNode(name); node.addGeom(geom)
    return NodePath(node)

def make_cylinder(name, radius, height, segs=12, color=(0.3,0.3,0.3,1)):
    """Vertical cylinder centered at origin."""
    vdata, v, n, c = _base_geom(name)
    tris = GeomTriangles(Geom.UHStatic)
    hh = height/2
    # Side faces
    for i in range(segs):
        a0 = 2*math.pi*i/segs; a1 = 2*math.pi*(i+1)/segs
        x0,y0 = radius*math.cos(a0),radius*math.sin(a0)
        x1,y1 = radius*math.cos(a1),radius*math.sin(a1)
        base_idx = v.getWriteRow()
        for x,y,z in [(x0,y0,-hh),(x1,y1,-hh),(x1,y1,hh),(x0,y0,hh)]:
            nx = math.cos((a0+a1)/2); ny = math.sin((a0+a1)/2)
            v.addData3(x,y,z); n.addData3(nx,ny,0); c.addData4(*color)
        tris.addVertices(base_idx,base_idx+1,base_idx+2)
        tris.addVertices(base_idx,base_idx+2,base_idx+3)
    geom = Geom(vdata); geom.addPrimitive(tris)
    node = GeomNode(name); node.addGeom(geom)
    return NodePath(node)

def make_line_quad(name, length, width=0.02, color=(0.1,0.9,0.1,1)):
    """A thin rectangular blade along X axis."""
    return make_box(name, length, width, 0.01, color)

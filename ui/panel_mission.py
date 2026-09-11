"""ui/panel_mission.py — Waypoint Mission Planner"""
import numpy as np
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QPushButton, QSizePolicy, QListWidget, QListWidgetItem, QDoubleSpinBox, QGridLayout)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QPainter, QColor, QPen, QFont, QMouseEvent
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from physics import compute_physics

# ── 2D Map Widget ─────────────────────────────────────────────────────────────
class MapWidget(QWidget):
    waypoint_added = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background:#0a0f17; border:1px solid #21262d; border-radius:8px;")
        self._waypoints = []   # list of (gx, gy) in world coords (-10..10)
        self._drag_idx = -1
        self.setMouseTracking(True)

    def _to_screen(self, gx, gy):
        w,h = self.width(), self.height()
        sx = int((gx+10)/20 * w)
        sy = int((1-(gy+10)/20) * h)
        return sx, sy

    def _to_world(self, sx, sy):
        w,h = self.width(), self.height()
        gx = sx/w * 20 - 10
        gy = (1 - sy/h) * 20 - 10
        return gx, gy

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w,h = self.width(), self.height()
        p.fillRect(0,0,w,h, QColor("#0a0f17"))

        # Grid
        pen = QPen(QColor("#1a2a3a")); pen.setWidth(1); p.setPen(pen)
        for i in range(0,21,2):
            sx,_ = self._to_screen(i-10, 0); _,sy = self._to_screen(0, i-10)
            p.drawLine(sx,0,sx,h); p.drawLine(0,sy,w,sy)

        # Axes
        pen2 = QPen(QColor("#21262d")); pen2.setWidth(2); p.setPen(pen2)
        sx0,_ = self._to_screen(0,0); _,sy0 = self._to_screen(0,0)
        p.drawLine(sx0,0,sx0,h); p.drawLine(0,sy0,w,sy0)

        # Route line
        if len(self._waypoints) > 1:
            pen3 = QPen(QColor("#1f6feb")); pen3.setWidth(2)
            pen3.setStyle(Qt.PenStyle.DashLine); p.setPen(pen3)
            for i in range(len(self._waypoints)-1):
                x1,y1 = self._to_screen(*self._waypoints[i][:2])
                x2,y2 = self._to_screen(*self._waypoints[i+1][:2])
                p.drawLine(x1,y1,x2,y2)

        # Home marker
        sx0,sy0 = self._to_screen(0,0)
        p.setBrush(QColor("#3fb950")); pen4=QPen(QColor("#238636")); pen4.setWidth(2); p.setPen(pen4)
        p.drawEllipse(sx0-8,sy0-8,16,16)
        p.setPen(QColor("white")); f=QFont("Consolas",7,QFont.Weight.Bold); p.setFont(f)
        p.drawText(sx0-4,sy0+4,"H")

        # Waypoints
        for i,(gx,gy,gz) in enumerate(self._waypoints):
            sx,sy = self._to_screen(gx,gy)
            p.setBrush(QColor("#58a6ff")); penw=QPen(QColor("#1f6feb")); penw.setWidth(2); p.setPen(penw)
            p.drawEllipse(sx-9,sy-9,18,18)
            p.setPen(QColor("white")); p.setFont(QFont("Consolas",8,QFont.Weight.Bold))
            lbl = str(i+1); p.drawText(sx-4,sy+4,lbl)

        p.end()

    def mousePressEvent(self, e: QMouseEvent):
        gx,gy = self._to_world(e.position().x(), e.position().y())
        if e.button() == Qt.MouseButton.LeftButton:
            # Check if clicking near existing WP
            for i,(wx,wy,_) in enumerate(self._waypoints):
                sx,sy = self._to_screen(wx,wy)
                if abs(e.position().x()-sx)<12 and abs(e.position().y()-sy)<12:
                    self._drag_idx = i; return
            # Add new WP
            self._waypoints.append((gx,gy,5.0))
            self.waypoint_added.emit(gx,gy)
            self.update()
        elif e.button() == Qt.MouseButton.RightButton:
            # Remove nearest WP
            best=-1; best_d=1e9
            for i,(wx,wy,_) in enumerate(self._waypoints):
                sx,sy = self._to_screen(wx,wy)
                d=((e.position().x()-sx)**2+(e.position().y()-sy)**2)**0.5
                if d<best_d: best_d=d; best=i
            if best>=0 and best_d<15:
                self._waypoints.pop(best); self.update()

    def mouseMoveEvent(self, e: QMouseEvent):
        if self._drag_idx>=0:
            gx,gy = self._to_world(e.position().x(), e.position().y())
            old = self._waypoints[self._drag_idx]
            self._waypoints[self._drag_idx] = (gx,gy,old[2])
            self.update()

    def mouseReleaseEvent(self, e):
        self._drag_idx = -1

    def get_waypoints(self): return list(self._waypoints)
    def clear_waypoints(self): self._waypoints=[]; self.update()
    def update_wp_alt(self, idx, alt):
        if 0<=idx<len(self._waypoints):
            x,y,_ = self._waypoints[idx]
            self._waypoints[idx]=(x,y,alt); self.update()


class MissionPanel(QWidget):
    fly_mission = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._params = None
        self._build_ui()

    def _build_ui(self):
        root = QHBoxLayout(self); root.setContentsMargins(12,12,12,12); root.setSpacing(12)

        # ── Left: Map ──
        left = QVBoxLayout()
        hdr=QHBoxLayout()
        t=QLabel("MISSION PLANNER"); t.setObjectName("section_title"); hdr.addWidget(t)
        hdr.addStretch()
        clr_btn=QPushButton("Clear All"); clr_btn.setObjectName("danger_btn"); clr_btn.clicked.connect(self._clear)
        hdr.addWidget(clr_btn); left.addLayout(hdr)
        hint=QLabel("Left-click: Add waypoint  |  Left-drag: Move waypoint  |  Right-click: Remove")
        hint.setObjectName("unit_lbl"); left.addWidget(hint)
        self._map = MapWidget(); left.addWidget(self._map,1)
        self._map.waypoint_added.connect(self._wp_added)
        lv = QVBoxLayout(); lv.addLayout(left)
        root.addLayout(lv,2)

        # ── Right: Waypoint list + stats ──
        right = QVBoxLayout(); right.setSpacing(8)
        t2=QLabel("WAYPOINTS"); t2.setObjectName("section_title"); right.addWidget(t2)
        self._wp_list = QListWidget()
        self._wp_list.setStyleSheet("background:#161b22;color:#e6edf3;border:1px solid #21262d;border-radius:6px;font-size:11px;")
        self._wp_list.setMaximumHeight(200); right.addWidget(self._wp_list)

        # Stats card
        stats_card=QFrame(); stats_card.setObjectName("card")
        sg=QGridLayout(stats_card); sg.setContentsMargins(16,12,16,12); sg.setSpacing(8)
        stat_items=[("Total Distance","_st_dist","m"),
                    ("Est. Flight Time","_st_time","min"),
                    ("Battery Required","_st_batt","Wh"),
                    ("Mission Feasible","_st_feas","")]
        for i,(n,attr,u) in enumerate(stat_items):
            nl=QLabel(n); nl.setObjectName("param_label"); sg.addWidget(nl,i,0)
            vl=QLabel("--"); vl.setObjectName("value_med"); sg.addWidget(vl,i,1)
            ul=QLabel(u); ul.setObjectName("unit_lbl"); sg.addWidget(ul,i,2)
            setattr(self,attr,vl)
        right.addWidget(stats_card)

        fly_btn=QPushButton("FLY THIS MISSION"); fly_btn.setObjectName("success_btn")
        fly_btn.clicked.connect(self._fly); right.addWidget(fly_btn)
        right.addStretch()
        root.addLayout(right,1)

    def _wp_added(self,gx,gy):
        wps=self._map.get_waypoints()
        self._wp_list.clear()
        for i,(x,y,z) in enumerate(wps):
            self._wp_list.addItem(f"WP{i+1}  X:{x:.1f}  Y:{y:.1f}  Alt:5m")
        self._update_stats()

    def _update_stats(self):
        wps=self._map.get_waypoints()
        if not wps: return
        dist=0.0
        pts=[(0,0,0)]+[(x,y,z) for x,y,z in wps]
        for i in range(len(pts)-1):
            d=np.sqrt((pts[i+1][0]-pts[i][0])**2+(pts[i+1][1]-pts[i][1])**2+(pts[i+1][2]-pts[i][2])**2)
            dist+=d
        self._st_dist.setText(f"{dist:.1f}")
        if self._params:
            p=self._params
            ph=compute_physics(p["m_payload"],p["m_battery"],p["batt_den"],
                               p["r_prop"],p["eta"],p["p_elec"],
                               p["altitude_m"],p["temp_c"],p["wind_ms"],
                               p["motor_wear"],p["batt_wear"])
            v_cruise=4.0
            t_flight=dist/v_cruise/60
            E_needed=ph["P_total"]*t_flight
            feasible=E_needed<=ph["E_batt_Wh"]*0.80
            self._st_time.setText(f"{t_flight:.1f}")
            self._st_batt.setText(f"{E_needed:.1f}")
            self._st_feas.setText("YES ✓" if feasible else "NO ✗")
            self._st_feas.setStyleSheet(f"color:{'#3fb950' if feasible else '#f85149'};font-weight:bold;font-size:13px;font-family:Consolas;")

    def set_params(self,p):
        self._params=p; self._update_stats()

    def _clear(self):
        self._map.clear_waypoints(); self._wp_list.clear()

    def _fly(self):
        wps=self._map.get_waypoints()
        if wps: self.fly_mission.emit([(x,y,z) for x,y,z in wps])

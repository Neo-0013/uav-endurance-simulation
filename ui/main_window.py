"""ui/main_window.py — Main Application Window"""
import os, sys
from PyQt6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QLabel, QStackedWidget, QStatusBar, QFrame)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QKeyEvent
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from ui.panel_home     import HomePanel
from ui.panel_sim      import SimPanel
from ui.panel_analysis import AnalysisPanel
from ui.panel_mission  import MissionPanel
from ui.panel_report   import ReportPanel

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV Endurance Simulation  |  v1.0")
        self.resize(1400,860)
        self._build_ui()
        self._nav_to(0)

    def _build_ui(self):
        root=QWidget(); self.setCentralWidget(root)
        layout=QHBoxLayout(root); layout.setContentsMargins(0,0,0,0); layout.setSpacing(0)

        # Sidebar
        sidebar=QWidget(); sidebar.setObjectName("sidebar"); sidebar.setFixedWidth(190)
        sv=QVBoxLayout(sidebar); sv.setContentsMargins(0,0,0,0); sv.setSpacing(0)
        tl=QLabel("UAV SIM"); tl.setObjectName("app_title"); sv.addWidget(tl)
        sl=QLabel("Endurance Dashboard"); sl.setObjectName("app_subtitle"); sv.addWidget(sl)
        sep=QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color:#21262d;margin:0 12px;"); sv.addWidget(sep)

        nav_items=[("Home / Config","home"),("3D Simulation","sim"),
                   ("Endurance Analysis","analysis"),("Mission Planner","mission"),
                   ("Report / Export","report")]
        icons=["  Config","  Simulate","  Analysis","  Mission","  Report"]
        self._nav_btns=[]
        for i,(label,_) in enumerate(nav_items):
            btn=QPushButton(f"{icons[i]}  {label}"); btn.setObjectName("nav_btn")
            btn.setProperty("active","false")
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            btn.clicked.connect(lambda _,idx=i: self._nav_to(idx))
            sv.addWidget(btn); self._nav_btns.append(btn)
        sv.addStretch()
        foot=QLabel("Physics: Momentum Theory\nController: PID"); foot.setObjectName("sidebar_footer")
        sv.addWidget(foot)
        layout.addWidget(sidebar)

        # QStackedWidget — ALL panels stay alive, sim timer never dies
        self._stack=QStackedWidget()
        self._panel_home=HomePanel()
        self._panel_sim=SimPanel()
        self._panel_analysis=AnalysisPanel()
        self._panel_mission=MissionPanel()
        self._panel_report=ReportPanel()
        for p in [self._panel_home,self._panel_sim,self._panel_analysis,
                  self._panel_mission,self._panel_report]:
            self._stack.addWidget(p)
        layout.addWidget(self._stack,1)

        self._statusbar=QStatusBar(); self.setStatusBar(self._statusbar)
        self._statusbar.showMessage("Ready")
        self._stimer=QTimer(self); self._stimer.setInterval(1000)
        self._stimer.timeout.connect(self._update_status); self._stimer.start()

        self._panel_home.params_changed.connect(self._on_params)
        self._panel_mission.fly_mission.connect(self._fly_mission)
        self._on_params(self._panel_home.get_params())

    def _nav_to(self,idx):
        self._stack.setCurrentIndex(idx)
        for i,btn in enumerate(self._nav_btns):
            btn.setProperty("active","true" if i==idx else "false")
            btn.style().unpolish(btn); btn.style().polish(btn)
        if idx==1: self._panel_sim.setFocus()

    def _on_params(self,params):
        self._panel_sim.set_params(params)
        self._panel_analysis.set_params(params)
        self._panel_mission.set_params(params)
        self._panel_report.set_params(params)

    def _fly_mission(self,waypoints):
        self._panel_sim.set_mission(waypoints)
        self._panel_sim._set_mode("autonomous")
        self._panel_sim._launch()
        self._nav_to(1)

    def _update_status(self):
        s=self._panel_sim
        if s._running:
            batt=(s._E/max(s._E0,1))*100
            self._statusbar.showMessage(
                f"SIM RUNNING  t={s._t:.1f}s  Alt={s._z:.1f}m  Batt={batt:.0f}%  Power={s._power:.1f}W")
        else:
            self._statusbar.showMessage("Sim idle  |  Navigate freely — simulation timer always active")

    def keyPressEvent(self,e:QKeyEvent):
        self._panel_sim.keyPressEvent(e)
    def keyReleaseEvent(self,e:QKeyEvent):
        self._panel_sim.keyReleaseEvent(e)

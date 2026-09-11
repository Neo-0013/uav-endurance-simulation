"""ui/panel_home.py — Configuration & Environment Panel"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QFrame, QGridLayout, QGroupBox, QScrollArea, QSizePolicy)
from PyQt6.QtCore import Qt, pyqtSignal
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from physics import compute_physics

def _make_sep():
    f = QFrame(); f.setFrameShape(QFrame.Shape.HLine)
    f.setStyleSheet("color:#21262d;"); return f

class SliderRow(QWidget):
    changed = pyqtSignal()
    def __init__(self, label, key, lo, hi, default, step, unit, decimals=2, parent=None):
        super().__init__(parent)
        self.key=key; self.lo=lo; self.hi=hi; self.step=step
        self.decimals=decimals; self.unit=unit
        self._scale = int(1/step) if step < 1 else 1

        h = QHBoxLayout(self); h.setContentsMargins(0,4,0,4)
        lbl = QLabel(label); lbl.setObjectName("param_label")
        lbl.setFixedWidth(200); h.addWidget(lbl)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(int(lo*self._scale), int(hi*self._scale))
        self.slider.setValue(int(default*self._scale))
        self.slider.setFixedWidth(260)
        h.addWidget(self.slider)

        self.val_lbl = QLabel(f"{default:.{decimals}f} {unit}")
        self.val_lbl.setObjectName("value_med")
        self.val_lbl.setFixedWidth(110)
        self.val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        h.addWidget(self.val_lbl)

        self.slider.valueChanged.connect(self._on_change)

    def _on_change(self, v):
        real = v / self._scale
        self.val_lbl.setText(f"{real:.{self.decimals}f} {self.unit}")
        self.changed.emit()

    def value(self):
        return self.slider.value() / self._scale


class HomePanel(QWidget):
    params_changed = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        root = QVBoxLayout(content); root.setContentsMargins(20,20,20,20); root.setSpacing(16)

        # ── Title ──
        t = QLabel("CONFIGURATION"); t.setObjectName("section_title"); root.addWidget(t)
        root.addWidget(_make_sep())

        # ── UAV Parameters ──
        uav_box = QGroupBox("UAV PARAMETERS"); vb = QVBoxLayout(uav_box)
        self.s_payload = SliderRow("Payload Mass",         "m_payload", 0.0, 3.0,  0.5,  0.05, "kg")
        self.s_battery = SliderRow("Battery Mass",         "m_battery", 0.2, 2.0,  0.8,  0.05, "kg")
        self.s_bden    = SliderRow("Battery Energy Density","batt_den",  80,  250,  160,  1,    "Wh/kg", 0)
        self.s_rprop   = SliderRow("Propeller Radius",     "r_prop",    0.06,0.25, 0.12, 0.01, "m")
        self.s_eta     = SliderRow("Motor Efficiency",     "eta",       0.30,0.90, 0.55, 0.01, "")
        self.s_pelec   = SliderRow("Electronics Power",    "p_elec",    5,   40,   15,   1,    "W", 0)
        for s in [self.s_payload,self.s_battery,self.s_bden,
                  self.s_rprop,self.s_eta,self.s_pelec]:
            vb.addWidget(s); s.changed.connect(self._emit)
        root.addWidget(uav_box)

        # ── Environment ──
        env_box = QGroupBox("ENVIRONMENT & CONDITIONS"); vb2 = QVBoxLayout(env_box)
        self.s_alt   = SliderRow("Altitude (ASL)",     "altitude",  0,   3000, 0,   50,  "m",  0)
        self.s_temp  = SliderRow("Ground Temperature", "temp_c",   -20,   50,  15,   1,  "°C", 0)
        self.s_wind  = SliderRow("Wind Speed",         "wind_ms",   0,    20,   0,  0.5, "m/s")
        for s in [self.s_alt,self.s_temp,self.s_wind]:
            vb2.addWidget(s); s.changed.connect(self._emit)
        root.addWidget(env_box)

        # ── Degradation ──
        deg_box = QGroupBox("COMPONENT DEGRADATION"); vb3 = QVBoxLayout(deg_box)
        self.s_mwear = SliderRow("Motor Wear",   "motor_wear",  0, 100, 0, 1, "%", 0)
        self.s_bwear = SliderRow("Battery Wear", "batt_wear",   0, 100, 0, 1, "%", 0)
        for s in [self.s_mwear,self.s_bwear]:
            vb3.addWidget(s); s.changed.connect(self._emit)
        root.addWidget(deg_box)

        root.addWidget(_make_sep())

        # ── Live Computed Values ──
        cv_lbl = QLabel("LIVE COMPUTED VALUES"); cv_lbl.setObjectName("section_title")
        root.addWidget(cv_lbl)

        cv_card = QFrame(); cv_card.setObjectName("card")
        grid = QGridLayout(cv_card); grid.setContentsMargins(16,16,16,16); grid.setSpacing(10)
        metrics = [
            ("Total Mass","_v_mass","kg"),("Total Weight","_v_weight","N"),
            ("Hover Thrust","_v_thrust","N"),("Air Density ρ","_v_rho","kg/m³"),
            ("Hover Power","_v_power","W"),("Battery Energy","_v_energy","Wh"),
            ("Endurance","_v_endur","min"),("Power / kg","_v_pwkg","W/kg"),
        ]
        for i,(name,attr,unit) in enumerate(metrics):
            r,c = divmod(i,2)
            col_off = c*3
            nl = QLabel(name); nl.setObjectName("param_label")
            vl = QLabel("--"); vl.setObjectName("value_med")
            ul = QLabel(unit); ul.setObjectName("unit_lbl")
            grid.addWidget(nl, r, col_off)
            grid.addWidget(vl, r, col_off+1)
            grid.addWidget(ul, r, col_off+2)
            setattr(self, attr, vl)
        root.addWidget(cv_card)
        root.addStretch()

        scroll.setWidget(content)
        outer = QVBoxLayout(self); outer.setContentsMargins(0,0,0,0)
        outer.addWidget(scroll)
        self._emit()

    def get_params(self):
        return {
            "m_payload":  self.s_payload.value(),
            "m_battery":  self.s_battery.value(),
            "batt_den":   self.s_bden.value(),
            "r_prop":     self.s_rprop.value(),
            "eta":        self.s_eta.value(),
            "p_elec":     self.s_pelec.value(),
            "altitude_m": self.s_alt.value(),
            "temp_c":     self.s_temp.value(),
            "wind_ms":    self.s_wind.value(),
            "motor_wear": self.s_mwear.value(),
            "batt_wear":  self.s_bwear.value(),
        }

    def _emit(self):
        p = self.get_params()
        ph = compute_physics(p["m_payload"],p["m_battery"],p["batt_den"],
                             p["r_prop"],p["eta"],p["p_elec"],
                             p["altitude_m"],p["temp_c"],p["wind_ms"],
                             p["motor_wear"],p["batt_wear"])
        self._v_mass.setText(f"{ph['m_total']:.3f}")
        self._v_weight.setText(f"{ph['weight']:.2f}")
        self._v_thrust.setText(f"{ph['thrust_hover']:.2f}")
        self._v_rho.setText(f"{ph['rho']:.4f}")
        self._v_power.setText(f"{ph['P_total']:.1f}")
        self._v_energy.setText(f"{ph['E_batt_Wh']:.1f}")
        self._v_endur.setText(f"{ph['endurance_m']:.1f}")
        self._v_pwkg.setText(f"{ph['P_total']/ph['m_total']:.1f}")
        self.params_changed.emit(p)

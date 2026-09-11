"""ui/panel_report.py — PDF Export Panel"""
import os, sys
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QFrame,
    QPushButton, QFileDialog, QTextEdit)
from PyQt6.QtCore import Qt
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

class ReportPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._params = None
        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self); root.setContentsMargins(32,24,32,24); root.setSpacing(16)
        t=QLabel("REPORT & EXPORT"); t.setObjectName("section_title"); root.addWidget(t)

        card=QFrame(); card.setObjectName("card")
        cv=QVBoxLayout(card); cv.setContentsMargins(24,20,24,20); cv.setSpacing(8)
        cv.addWidget(QLabel("PDF Report will contain:"))
        for item in ["Cover page with project title and date",
                     "UAV configuration parameters table",
                     "Computed physics results (mass, thrust, power, endurance)",
                     "Momentum Theory physics background",
                     "Endurance vs Payload chart",
                     "Endurance vs Power chart",
                     "Optimal operating condition analysis"]:
            l=QLabel(f"  \u2713  {item}"); l.setObjectName("param_label"); cv.addWidget(l)
        root.addWidget(card)

        self._log=QTextEdit(); self._log.setReadOnly(True); self._log.setMaximumHeight(100)
        self._log.setStyleSheet("background:#161b22;color:#8b949e;border:1px solid #21262d;border-radius:6px;font-family:Consolas;font-size:10px;")
        root.addWidget(self._log)

        btn=QPushButton("EXPORT PDF REPORT"); btn.setObjectName("primary_btn")
        btn.setMinimumHeight(48); btn.clicked.connect(self._export); root.addWidget(btn)
        root.addStretch()

    def set_params(self, p): self._params = p

    def _export(self):
        if not self._params:
            self._log.append("ERROR: Set parameters in Home panel first."); return
        path,_=QFileDialog.getSaveFileName(self,"Save PDF","UAV_Endurance_Report.pdf","PDF Files (*.pdf)")
        if not path: return
        self._log.append("Generating PDF...")
        try:
            p=self._params
            from physics import compute_physics, sweep_payload
            ph=compute_physics(p["m_payload"],p["m_battery"],p["batt_den"],
                               p["r_prop"],p["eta"],p["p_elec"],
                               p["altitude_m"],p["temp_c"],p["wind_ms"],
                               p["motor_wear"],p["batt_wear"])
            sw=sweep_payload(p["m_battery"],p["batt_den"],p["r_prop"],p["eta"],p["p_elec"],
                             p["altitude_m"],p["temp_c"],p["wind_ms"],p["motor_wear"],p["batt_wear"])
            from export.report import generate_pdf
            generate_pdf(path,p,ph,sw)
            self._log.append(f"SUCCESS: Saved to {path}")
        except Exception as ex:
            self._log.append(f"ERROR: {ex}")

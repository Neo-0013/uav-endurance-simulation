"""ui/panel_analysis.py — Endurance Analysis Panel"""
import numpy as np
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QPushButton, QScrollArea, QSizePolicy)
from PyQt6.QtCore import Qt
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import warnings; warnings.filterwarnings("ignore")
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from physics import compute_physics, sweep_payload, air_density

DARK="#0d1117"; PANEL="#161b22"; CYAN="#58a6ff"; GREEN="#3fb950"
ORANGE="#d29922"; RED="#f85149"; GREY="#444466"; MUTED="#6e7681"

def style_ax(ax, title, xl, yl):
    ax.set_facecolor(PANEL); ax.set_title(title,color=CYAN,fontsize=9,fontweight="bold",pad=6)
    ax.set_xlabel(xl,color=MUTED,fontsize=8); ax.set_ylabel(yl,color=MUTED,fontsize=8)
    ax.tick_params(colors=GREY,labelsize=7); ax.spines[:].set_color("#21262d")
    ax.grid(True,color="#21262d",lw=0.5,linestyle="--")

class AnalysisPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._params = None
        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self); root.setContentsMargins(12,12,12,12); root.setSpacing(8)
        hdr = QHBoxLayout()
        t = QLabel("ENDURANCE ANALYSIS"); t.setObjectName("section_title"); hdr.addWidget(t)
        hdr.addStretch()
        btn = QPushButton("Refresh Analysis"); btn.setObjectName("primary_btn")
        btn.clicked.connect(self._run); hdr.addWidget(btn)
        root.addLayout(hdr)

        self._fig = plt.Figure(figsize=(12,7), facecolor=DARK)
        self._canvas = FigureCanvas(self._fig)
        self._canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        root.addWidget(self._canvas)

        # Optimal condition card
        opt_card = QFrame(); opt_card.setObjectName("optimal_box")
        oh = QHBoxLayout(opt_card); oh.setContentsMargins(16,12,16,12)
        ot = QLabel("OPTIMAL OPERATING CONDITION"); ot.setObjectName("optimal_title"); oh.addWidget(ot)
        oh.addStretch()
        self._opt_labels = {}
        for key in ["Payload","Endurance","Power","Altitude","Wind"]:
            vb = QVBoxLayout()
            nl = QLabel(key.upper()); nl.setObjectName("unit_lbl"); vb.addWidget(nl)
            vl = QLabel("--"); vl.setObjectName("optimal_value"); vb.addWidget(vl)
            self._opt_labels[key] = vl; oh.addLayout(vb)
        root.addWidget(opt_card)

        self._placeholder()

    def _placeholder(self):
        self._fig.clf()
        ax = self._fig.add_subplot(111); ax.set_facecolor(DARK)
        ax.text(0.5,0.5,"Click 'Refresh Analysis' to generate plots",
                ha="center",va="center",color=MUTED,fontsize=13,transform=ax.transAxes)
        ax.axis("off"); self._canvas.draw()

    def set_params(self, params):
        self._params = params
        self._run()

    def _run(self):
        if self._params is None: return
        p = self._params
        ph = compute_physics(p["m_payload"],p["m_battery"],p["batt_den"],
                             p["r_prop"],p["eta"],p["p_elec"],
                             p["altitude_m"],p["temp_c"],p["wind_ms"],
                             p["motor_wear"],p["batt_wear"])
        payload_range = np.linspace(0,3.0,120)
        data = sweep_payload(p["m_battery"],p["batt_den"],p["r_prop"],p["eta"],p["p_elec"],
                             p["altitude_m"],p["temp_c"],p["wind_ms"],p["motor_wear"],p["batt_wear"],
                             payload_range)
        payloads=data[:,0]; powers=data[:,1]; endurs=data[:,2]

        self._fig.clf(); self._fig.patch.set_facecolor(DARK)
        gs = gridspec.GridSpec(2,3,figure=self._fig,hspace=0.5,wspace=0.4,
                               left=0.06,right=0.97,top=0.91,bottom=0.09)
        ax1=self._fig.add_subplot(gs[0,:2])
        ax2=self._fig.add_subplot(gs[1,:2])
        ax3=self._fig.add_subplot(gs[0,2])
        ax4=self._fig.add_subplot(gs[1,2])

        cur_e=ph["endurance_m"]; cur_mp=p["m_payload"]
        target=15.0; good=np.where(endurs>=target)[0]
        sp_load=sp_end=sp_pwr=0.0; si=-1
        if len(good)>0:
            si=good[-1]; sp_load=payloads[si]; sp_end=endurs[si]; sp_pwr=powers[si]

        # ── Plot 1: Endurance vs Payload ──
        style_ax(ax1,"FLIGHT ENDURANCE  vs  PAYLOAD MASS","Payload Mass (kg)","Endurance (min)")
        ax1.plot(payloads,endurs,color=CYAN,lw=2.2,zorder=3)
        ax1.fill_between(payloads,endurs,alpha=0.10,color=CYAN)
        ax1.plot(cur_mp,cur_e,"o",color=GREEN,ms=8,zorder=5,
                 label=f"Current: {cur_mp:.2f} kg → {cur_e:.1f} min")
        if si>=0:
            ax1.axvline(sp_load,color=ORANGE,lw=1.2,ls="--",alpha=0.7)
            ax1.axhline(target,color=ORANGE,lw=1.0,ls=":",alpha=0.6)
            ax1.plot(sp_load,sp_end,"*",color=ORANGE,ms=14,zorder=6,
                     label=f"Sweet Spot: {sp_load:.2f} kg @ {sp_end:.1f} min")
        ax1.legend(fontsize=7.5,facecolor="#112233",edgecolor=GREY,labelcolor="white",loc="upper right")
        ax1.set_xlim(0,3.0); ax1.set_ylim(bottom=0)

        # ── Plot 2: Endurance vs Power ──
        style_ax(ax2,"FLIGHT ENDURANCE  vs  POWER CONSUMPTION","Total Power (W)","Endurance (min)")
        si2=np.argsort(powers)
        ax2.plot(powers[si2],endurs[si2],color=RED,lw=2.2,zorder=3)
        ax2.fill_between(powers[si2],endurs[si2],alpha=0.10,color=RED)
        ax2.plot(ph["P_total"],cur_e,"o",color=GREEN,ms=8,zorder=5,
                 label=f"Current: {ph['P_total']:.1f} W → {cur_e:.1f} min")
        ax2.legend(fontsize=7.5,facecolor="#112233",edgecolor=GREY,labelcolor="white")

        # ── Plot 3: Power Breakdown Pie ──
        ax3.set_facecolor(DARK); ax3.axis("off")
        pos3=ax3.get_position()
        pie_ax=self._fig.add_axes([pos3.x0,pos3.y0+pos3.height*0.05,pos3.width,pos3.height*0.90])
        pie_ax.set_facecolor(DARK)
        sizes=[ph["P_prop"],ph["P_wind"],ph["P_elec"]]
        lbls=[f"Propulsion\n{ph['P_prop']:.1f}W",f"Wind Drag\n{ph['P_wind']:.1f}W",f"Electronics\n{ph['P_elec']:.1f}W"]
        clrs=[CYAN,ORANGE,GREEN]
        if ph["P_wind"]<0.05: sizes=sizes[:1]+sizes[2:]; lbls=lbls[:1]+lbls[2:]; clrs=clrs[:1]+clrs[2:]
        wedges,texts,autos=pie_ax.pie(sizes,labels=lbls,colors=clrs,explode=[0.05]*len(sizes),
                                       autopct="%1.1f%%",startangle=90,
                                       textprops={"color":"white","fontsize":7},
                                       wedgeprops={"lw":2,"edgecolor":DARK})
        for at in autos: at.set_color(DARK); at.set_fontweight("bold"); at.set_fontsize(7)
        pie_ax.set_title("POWER BREAKDOWN",color=CYAN,fontsize=9,fontweight="bold",pad=4)

        # ── Plot 4: Endurance vs Wind Speed ──
        style_ax(ax4,"ENDURANCE vs WIND SPEED","Wind Speed (m/s)","Endurance (min)")
        winds=np.linspace(0,20,20)
        ends_w=[compute_physics(p["m_payload"],p["m_battery"],p["batt_den"],
                                p["r_prop"],p["eta"],p["p_elec"],
                                p["altitude_m"],p["temp_c"],w,
                                p["motor_wear"],p["batt_wear"])["endurance_m"] for w in winds]
        ax4.plot(winds,ends_w,color="#bc8cff",lw=2.0)
        ax4.axvline(p["wind_ms"],color=ORANGE,lw=1.2,ls="--",alpha=0.7,label=f"Current {p['wind_ms']:.1f}m/s")
        ax4.fill_between(winds,ends_w,alpha=0.10,color="#bc8cff")
        ax4.legend(fontsize=7,facecolor="#112233",edgecolor=GREY,labelcolor="white")
        ax4.set_xlim(0,20); ax4.set_ylim(bottom=0)

        self._fig.suptitle(
            f"UAV ENDURANCE ANALYSIS  |  r={p['r_prop']:.2f}m  η={p['eta']:.2f}  "
            f"Batt={p['m_battery']:.1f}kg×{p['batt_den']:.0f}Wh/kg  "
            f"Alt={p['altitude_m']:.0f}m  Temp={p['temp_c']:.0f}°C",
            color=CYAN,fontsize=9,y=0.97)
        self._canvas.draw()

        # Update optimal card
        self._opt_labels["Payload"].setText(f"{sp_load:.2f} kg" if si>=0 else "N/A")
        self._opt_labels["Endurance"].setText(f"{sp_end:.1f} min" if si>=0 else "N/A")
        self._opt_labels["Power"].setText(f"{sp_pwr:.1f} W" if si>=0 else "N/A")
        self._opt_labels["Altitude"].setText(f"{p['altitude_m']:.0f} m")
        self._opt_labels["Wind"].setText(f"{p['wind_ms']:.1f} m/s")

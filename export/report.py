"""export/report.py — PDF Report Generator"""
import io, datetime
import numpy as np
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import cm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
    TableStyle, Image, HRFlowable, PageBreak)
from reportlab.lib.enums import TA_CENTER
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DARK="#0d1117"; PANEL="#161b22"; BLUE="#1f6feb"; GREEN="#238636"; TEAL="#58a6ff"
CYAN="#58a6ff"; ORANGE="#d29922"; RED="#f85149"; MUTED="#8b949e"

def _fig_img(fig, w=16, h=7):
    buf = io.BytesIO()
    fig.savefig(buf,format="png",dpi=150,bbox_inches="tight",facecolor=fig.get_facecolor())
    buf.seek(0)
    return Image(buf,width=w*cm,height=h*cm)

def generate_pdf(path, params, physics, sweep_data):
    doc = SimpleDocTemplate(path, pagesize=A4,
          leftMargin=2*cm,rightMargin=2*cm,topMargin=2*cm,bottomMargin=2*cm)
    styles = getSampleStyleSheet()
    H1=ParagraphStyle("H1",fontSize=17,textColor=colors.HexColor(TEAL),
                       spaceAfter=6,fontName="Helvetica-Bold",alignment=TA_CENTER)
    H2=ParagraphStyle("H2",fontSize=12,textColor=colors.HexColor(TEAL),
                       spaceAfter=4,fontName="Helvetica-Bold")
    BODY=ParagraphStyle("BODY",fontSize=9,textColor=colors.HexColor("#c9d1d9"),
                         spaceAfter=4,leading=14)
    story=[]
    story.append(Spacer(1,1.5*cm))
    story.append(Paragraph("UAV QUADCOPTER ENDURANCE SIMULATION",H1))
    story.append(Paragraph("Aerodynamic Analysis & Endurance Report",H1))
    story.append(Spacer(1,0.4*cm))
    story.append(HRFlowable(width="100%",color=colors.HexColor(BLUE),thickness=1.5))
    story.append(Spacer(1,0.3*cm))
    story.append(Paragraph(f"Generated: {datetime.datetime.now().strftime('%B %d, %Y  %H:%M')}",BODY))
    story.append(Spacer(1,0.8*cm))

    story.append(Paragraph("1. UAV CONFIGURATION PARAMETERS",H2))
    story.append(HRFlowable(width="100%",color=colors.HexColor("#21262d"),thickness=0.5))
    story.append(Spacer(1,0.2*cm))
    tdata=[["Parameter","Value","Unit"],
           ["Empty Airframe Mass","1.20","kg"],
           ["Payload Mass",f"{params['m_payload']:.2f}","kg"],
           ["Battery Mass",f"{params['m_battery']:.2f}","kg"],
           ["Battery Energy Density",f"{params['batt_den']:.0f}","Wh/kg"],
           ["Propeller Radius",f"{params['r_prop']:.3f}","m"],
           ["Motor Efficiency",f"{params['eta']*100:.0f}","%"],
           ["Electronics Power",f"{params['p_elec']:.0f}","W"],
           ["Altitude (ASL)",f"{params['altitude_m']:.0f}","m"],
           ["Wind Speed",f"{params['wind_ms']:.1f}","m/s"],
           ["Motor Wear",f"{params['motor_wear']:.0f}","%"],
           ["Battery Degradation",f"{params['batt_wear']:.0f}","%"]]
    ts=TableStyle([("BACKGROUND",(0,0),(-1,0),colors.HexColor(BLUE)),
                   ("TEXTCOLOR",(0,0),(-1,0),colors.white),
                   ("FONTNAME",(0,0),(-1,0),"Helvetica-Bold"),
                   ("FONTSIZE",(0,0),(-1,-1),9),
                   ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.HexColor(PANEL),colors.HexColor(DARK)]),
                   ("TEXTCOLOR",(0,1),(-1,-1),colors.HexColor("#c9d1d9")),
                   ("GRID",(0,0),(-1,-1),0.5,colors.HexColor("#21262d")),
                   ("ALIGN",(1,0),(-1,-1),"CENTER"),
                   ("TOPPADDING",(0,0),(-1,-1),4),("BOTTOMPADDING",(0,0),(-1,-1),4)])
    t=Table(tdata,colWidths=[7*cm,4*cm,3*cm]); t.setStyle(ts)
    story.append(t); story.append(Spacer(1,0.5*cm))

    story.append(Paragraph("2. COMPUTED RESULTS",H2))
    story.append(HRFlowable(width="100%",color=colors.HexColor("#21262d"),thickness=0.5))
    story.append(Spacer(1,0.2*cm))
    rdata=[["Quantity","Value","Unit"],
           ["Total Mass",f"{physics['m_total']:.3f}","kg"],
           ["Total Weight",f"{physics['weight']:.2f}","N"],
           ["Air Density (rho)",f"{physics['rho']:.4f}","kg/m3"],
           ["Hover Thrust",f"{physics['thrust_hover']:.2f}","N"],
           ["Induced Power",f"{physics['P_induced']:.2f}","W"],
           ["Wind Drag Power",f"{physics['P_wind']:.2f}","W"],
           ["Total Hover Power",f"{physics['P_total']:.2f}","W"],
           ["Battery Energy (usable)",f"{physics['E_batt_Wh']*0.8:.1f}","Wh"],
           ["Flight Endurance",f"{physics['endurance_m']:.2f}","min"]]
    ts2=TableStyle([("BACKGROUND",(0,0),(-1,0),colors.HexColor(GREEN)),
                    ("TEXTCOLOR",(0,0),(-1,0),colors.white),
                    ("FONTNAME",(0,0),(-1,0),"Helvetica-Bold"),
                    ("FONTSIZE",(0,0),(-1,-1),9),
                    ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.HexColor(PANEL),colors.HexColor(DARK)]),
                    ("TEXTCOLOR",(0,1),(-1,-1),colors.HexColor("#c9d1d9")),
                    ("GRID",(0,0),(-1,-1),0.5,colors.HexColor("#21262d")),
                    ("ALIGN",(1,0),(-1,-1),"CENTER"),
                    ("TOPPADDING",(0,0),(-1,-1),4),("BOTTOMPADDING",(0,0),(-1,-1),4)])
    t2=Table(rdata,colWidths=[7*cm,4*cm,3*cm]); t2.setStyle(ts2)
    story.append(t2); story.append(Spacer(1,0.4*cm))

    story.append(Paragraph("3. PHYSICS BACKGROUND",H2))
    story.append(HRFlowable(width="100%",color=colors.HexColor("#21262d"),thickness=0.5))
    story.append(Spacer(1,0.2*cm))
    story.append(Paragraph(
        "The endurance model uses <b>Actuator Disk / Momentum Theory</b>. "
        "In hover: Thrust = Weight = m_total x g. Induced power: "
        "<b>P = T^(3/2) / sqrt(2 x rho x A_disk)</b>. "
        "Total power = P/eta + P_wind + P_electronics. "
        "<b>Endurance = 0.8 x E_battery / P_total</b> (80% LiPo discharge limit).",BODY))
    story.append(PageBreak())

    story.append(Paragraph("4. ENDURANCE ANALYSIS CHARTS",H2))
    story.append(HRFlowable(width="100%",color=colors.HexColor("#21262d"),thickness=0.5))
    story.append(Spacer(1,0.3*cm))
    payloads=sweep_data[:,0]; powers=sweep_data[:,1]; endurs=sweep_data[:,2]
    target=15.0; good=np.where(endurs>=target)[0]
    sp_load=sp_end=0.0; sp_pwr=0.0
    if len(good)>0:
        si=good[-1]; sp_load=payloads[si]; sp_end=endurs[si]; sp_pwr=powers[si]

    fig,axes=plt.subplots(1,2,figsize=(14,5),facecolor=DARK)
    for ax,xs,ys,color,xl,yl,ttl in [
        (axes[0],payloads,endurs,CYAN,"Payload (kg)","Endurance (min)","Endurance vs Payload"),
        (axes[1],sorted(powers),sorted(endurs,reverse=True),RED,"Total Power (W)","Endurance (min)","Endurance vs Power"),
    ]:
        ax.set_facecolor(PANEL); ax.set_title(ttl,color=CYAN,fontsize=11,fontweight="bold")
        ax.set_xlabel(xl,color=MUTED,fontsize=9); ax.set_ylabel(yl,color=MUTED,fontsize=9)
        ax.tick_params(colors="#444466",labelsize=8); ax.spines[:].set_color("#21262d")
        ax.grid(True,color="#21262d",lw=0.6,ls="--")
        ax.plot(xs,ys,color=color,lw=2.5); ax.fill_between(xs,ys,alpha=0.12,color=color)
    if len(good)>0:
        axes[0].axvline(sp_load,color=ORANGE,lw=1.2,ls="--",alpha=0.8)
        axes[0].axhline(target,color=ORANGE,lw=1.0,ls=":",alpha=0.7)
        axes[0].plot(sp_load,sp_end,"*",color=ORANGE,ms=16,zorder=6,
                     label=f"Sweet Spot: {sp_load:.2f}kg @ {sp_end:.1f}min")
        axes[0].legend(fontsize=8,facecolor="#112233",labelcolor="white",edgecolor="#21262d")
    axes[0].plot(params["m_payload"],physics["endurance_m"],"o",color="#3fb950",ms=10,zorder=5)
    fig.tight_layout(pad=1.5)
    story.append(_fig_img(fig,16,7)); plt.close(fig)
    story.append(Spacer(1,0.4*cm))

    story.append(Paragraph("5. OPTIMAL OPERATING CONDITION",H2))
    story.append(HRFlowable(width="100%",color=colors.HexColor("#21262d"),thickness=0.5))
    story.append(Spacer(1,0.2*cm))
    if len(good)>0:
        story.append(Paragraph(
            f"The most suitable operating condition is the maximum payload that guarantees "
            f"at least 15 minutes of flight: <b>Payload = {sp_load:.2f} kg, "
            f"Endurance = {sp_end:.1f} min, Power = {sp_pwr:.1f} W</b>. "
            f"Beyond this, the P proportional to T^1.5 relationship causes exponential battery drain.",BODY))
    else:
        story.append(Paragraph("With current parameters, endurance is below 15 min even at zero payload. Consider a larger battery.",BODY))
    doc.build(story)

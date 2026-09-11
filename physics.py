"""
physics.py — UAV Physics Engine
Momentum Theory hover model + ISA atmosphere + wind drag + degradation
"""
import numpy as np

G       = 9.81
N_PROPS = 4
M_EMPTY = 1.2   # fixed airframe mass (kg)

# ── ISA Standard Atmosphere ──────────────────────────────────────────────────
def air_density(altitude_m: float, temp_c: float = 15.0) -> float:
    """
    Compute air density using ISA model with optional ground temperature override.
    altitude_m : metres above sea level
    temp_c     : ground-level temperature in Celsius
    returns    : rho (kg/m^3)
    """
    T0 = temp_c + 273.15          # ground temperature in Kelvin
    L  = 0.0065                   # lapse rate (K/m)
    T  = T0 - L * altitude_m      # temperature at altitude
    T  = max(T, 216.65)           # tropopause floor
    rho0 = 1.225 * (T0 / 288.15)  # correct sea-level density for temp
    rho  = rho0 * (T / T0) ** 5.2561
    return float(rho)

# ── Core Hover Physics ───────────────────────────────────────────────────────
def compute_physics(
    m_payload: float,
    m_battery: float,
    batt_density: float,
    r_prop: float,
    eta: float,
    p_elec: float,
    altitude_m: float = 0.0,
    temp_c: float = 15.0,
    wind_ms: float = 0.0,
    motor_wear: float = 0.0,
    batt_wear: float = 0.0,
) -> dict:
    """
    Full physics computation for hover endurance.

    Parameters
    ----------
    m_payload    kg   payload mass
    m_battery    kg   battery mass
    batt_density Wh/kg battery energy density
    r_prop       m    propeller radius
    eta          -    nominal motor efficiency (0–1)
    p_elec       W    fixed electronics power
    altitude_m   m    flight altitude (affects rho)
    temp_c       °C   ground temperature
    wind_ms      m/s  horizontal wind speed
    motor_wear   %    motor degradation 0–100
    batt_wear    %    battery cycle degradation 0–100

    Returns
    -------
    dict with all computed values
    """
    rho = air_density(altitude_m, temp_c)

    # Degradation corrections
    eta_eff   = eta   * (1.0 - 0.30 * motor_wear / 100.0)
    E_factor  = 1.0   - 0.20 * batt_wear  / 100.0

    m_total   = M_EMPTY + m_battery + m_payload
    weight    = m_total * G
    A_disk    = N_PROPS * np.pi * r_prop ** 2

    # Hover thrust = weight
    T         = weight
    P_induced = (T ** 1.5) / np.sqrt(2.0 * rho * A_disk)
    P_prop    = P_induced / eta_eff

    # Wind drag penalty
    Cd_body   = 0.5
    A_frontal = 0.04   # m^2 representative frontal area
    F_drag    = 0.5 * rho * Cd_body * A_frontal * wind_ms ** 2
    P_wind    = F_drag * wind_ms  # extra power to maintain position

    P_total   = P_prop + P_wind + p_elec

    E_batt_Wh = m_battery * batt_density * E_factor
    endurance_m = (0.80 * E_batt_Wh) / P_total * 60.0 if P_total > 0 else 0.0

    return {
        "m_total":       m_total,
        "weight":        weight,
        "thrust_hover":  T,
        "rho":           rho,
        "P_induced":     P_induced,
        "P_prop":        P_prop,
        "P_wind":        P_wind,
        "P_elec":        p_elec,
        "P_total":       P_total,
        "E_batt_Wh":     E_batt_Wh,
        "endurance_m":   endurance_m,
        "A_disk":        A_disk,
        "eta_eff":       eta_eff,
        # store inputs for re-use
        "_rho":          rho,
        "_eta_eff":      eta_eff,
        "_r_prop":       r_prop,
    }

# ── Payload Sweep ────────────────────────────────────────────────────────────
def sweep_payload(
    m_battery, batt_density, r_prop, eta, p_elec,
    altitude_m=0.0, temp_c=15.0, wind_ms=0.0,
    motor_wear=0.0, batt_wear=0.0,
    payload_range=None,
):
    if payload_range is None:
        payload_range = np.linspace(0.0, 3.0, 120)
    rows = []
    for mp in payload_range:
        ph = compute_physics(mp, m_battery, batt_density, r_prop, eta, p_elec,
                             altitude_m, temp_c, wind_ms, motor_wear, batt_wear)
        rows.append((mp, ph["P_total"], ph["endurance_m"]))
    return np.array(rows)   # cols: payload, total_power, endurance_min

# ── PID Flight Simulation ────────────────────────────────────────────────────
def simulate_flight(params: dict, z_target=12.0, dt=0.04, t_end=20.0):
    """
    PID-controlled vertical takeoff + hover.
    Returns: t, z, vz, ax, ay, az_arr, thrust, power, battery_J arrays
    """
    m    = params["m_total"]
    A    = params["A_disk"]
    eta  = params["_eta_eff"]
    rho  = params["_rho"]
    pe   = params["P_elec"]
    E0   = params["E_batt_Wh"] * 3600 * 0.80
    Tmax = m * G * 2.8

    Kp, Ki, Kd = 5.0, 0.20, 4.0
    steps   = int(t_end / dt) + 1
    t_a  = np.zeros(steps); z_a  = np.zeros(steps)
    vz_a = np.zeros(steps); T_a  = np.zeros(steps)
    P_a  = np.zeros(steps); E_a  = np.zeros(steps)
    E_a[0] = E0
    integral = 0.0

    for i in range(steps - 1):
        t_a[i+1] = t_a[i] + dt
        err       = z_target - z_a[i]
        integral += err * dt
        deriv     = -vz_a[i]
        a_req     = Kp*err + Ki*integral + Kd*deriv
        T_req     = m * (a_req + G)
        T_a[i]   = float(np.clip(T_req, 0.0, Tmax))
        az        = T_a[i]/m - G
        vz_a[i+1]= vz_a[i] + az * dt
        z_a[i+1] = max(0.0, z_a[i] + vz_a[i+1]*dt)
        if z_a[i+1] == 0.0: vz_a[i+1] = 0.0
        Pi       = (T_a[i]**1.5 / np.sqrt(2*rho*A)) / eta + pe
        P_a[i]   = Pi
        E_a[i+1] = max(0.0, E_a[i] - Pi*dt)
    T_a[-1] = T_a[-2]; P_a[-1] = P_a[-2]
    return t_a, z_a, vz_a, T_a, P_a, E_a

# ── Autonomous Mission Simulation ────────────────────────────────────────────
def simulate_mission(params: dict, waypoints: list, dt=0.04):
    """
    Fly through a list of (x,y,z) waypoints.
    Returns: t, x, y, z, thrust, power, battery_J arrays
    """
    if not waypoints:
        return [np.array([0.0])]*7

    m    = params["m_total"]
    A    = params["A_disk"]
    eta  = params["_eta_eff"]
    rho  = params["_rho"]
    pe   = params["P_elec"]
    E0   = params["E_batt_Wh"] * 3600 * 0.80
    Tmax = m * G * 2.8
    v_max = 4.0   # m/s cruise speed

    t_list=[0.0]; x_list=[0.0]; y_list=[0.0]; z_list=[0.0]
    T_list=[0.0]; P_list=[0.0]; E_list=[E0]

    px, py, pz = 0.0, 0.0, 0.0
    vx, vy, vz = 0.0, 0.0, 0.0
    t = 0.0

    for wp in waypoints:
        tx, ty, tz = float(wp[0]), float(wp[1]), float(wp[2])
        for _ in range(int(30/dt)):   # max 30s per waypoint
            dx=tx-px; dy=ty-py; dz=tz-pz
            dist = np.sqrt(dx**2+dy**2+dz**2)
            if dist < 0.3: break
            # proportional velocity toward waypoint
            speed = min(v_max, dist*1.5)
            scale = speed/dist
            vx = dx*scale; vy = dy*scale; vz_cmd = dz*scale
            # simple position update
            px += vx*dt; py += vy*dt; pz += vz_cmd*dt
            pz = max(0.0, pz)
            T_hover = m*G
            Pi = (T_hover**1.5/np.sqrt(2*rho*A))/eta + pe
            prev_E = E_list[-1]
            E_new  = max(0.0, prev_E - Pi*dt)
            t += dt
            t_list.append(t); x_list.append(px); y_list.append(py); z_list.append(pz)
            T_list.append(T_hover); P_list.append(Pi); E_list.append(E_new)
            if E_new == 0.0: break
        if E_list[-1] == 0.0: break

    return (np.array(t_list), np.array(x_list), np.array(y_list),
            np.array(z_list), np.array(T_list), np.array(P_list), np.array(E_list))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# 1. 3D UAV Aerodynamic Object Visualization
# ==========================================
def plot_3d_uav():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # UAV Central Body (Sphere/Ellipsoid)
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x_body = 0.1 * np.cos(u) * np.sin(v)
    y_body = 0.1 * np.sin(u) * np.sin(v)
    z_body = 0.05 * np.cos(v)
    ax.plot_surface(x_body, y_body, z_body, color='b', alpha=0.6, label='Central Hub')
    
    # UAV Arms (Quad-copter 'X' configuration)
    arm_length = 0.3
    arms_x = [[0, arm_length], [0, -arm_length], [0, arm_length], [0, -arm_length]]
    arms_y = [[0, arm_length], [0, -arm_length], [0, -arm_length], [0, arm_length]]
    arms_z = [[0, 0], [0, 0], [0, 0], [0, 0]]
    
    for i in range(4):
        ax.plot(arms_x[i], arms_y[i], arms_z[i], color='k', linewidth=4)
        
    # Rotors (Disks at the end of arms)
    r_prop = 0.12
    for i in range(4):
        cx, cy = arms_x[i][1], arms_y[i][1]
        theta = np.linspace(0, 2*np.pi, 20)
        x_prop = cx + r_prop * np.cos(theta)
        y_prop = cy + r_prop * np.sin(theta)
        z_prop = np.zeros_like(theta) + 0.02
        # Fill the rotor disk
        ax.plot_trisurf(x_prop, y_prop, z_prop, color='g', alpha=0.5)

    ax.set_title('3D Representation of a Simple Quadcopter UAV', fontsize=14)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    
    # Equal aspect ratio using scaling limits
    ax.set_xlim([-0.4, 0.4])
    ax.set_ylim([-0.4, 0.4])
    ax.set_zlim([-0.4, 0.4])
    plt.tight_layout()
    plt.show()

# ==========================================
# 2. UAV Endurance & Power Model
# ==========================================
def calculate_and_plot_endurance():
    # --- UAV Parameters ---
    g = 9.81              # Gravity (m/s^2)
    rho = 1.225           # Air density at sea level (kg/m^3)
    
    m_empty = 1.2         # Empty weight of UAV (kg)
    m_battery = 0.8       # Battery weight (kg)
    
    # Battery specifications
    battery_energy_density = 160  # Wh/kg
    E_batt_Wh = m_battery * battery_energy_density # Total Battery Energy in Wh
    
    # Aerodynamic / Propulsion efficiency
    R_prop = 0.12         # Propeller radius (m)
    num_props = 4
    A_total = num_props * (np.pi * R_prop**2)  # Total actuator disk area (m^2)
    eta_total = 0.55      # Overall propulsion efficiency (propeller + motor + ESC)
    P_electronics = 15.0  # Power consumed by flight controller & payload sensors (W)
    
    # --- Range of Payload Mass to test ---
    # Testing payload from 0 kg up to 2.5 kg
    m_payload = np.linspace(0, 2.5, 50)
    
    # 1. Total Mass & Weight
    m_total = m_empty + m_battery + m_payload
    W_total = m_total * g
    
    # 2. Power Consumption (Hovering condition using Momentum Theory)
    # Ideal induced power: P_ideal = T^(3/2) / sqrt(2 * rho * A)
    # Total power: P_total = P_ideal / eta_total + P_electronics
    T = W_total # Thrust equals weight in hover
    P_ideal = (T**(1.5)) / np.sqrt(2 * rho * A_total)
    P_total = (P_ideal / eta_total) + P_electronics
    
    # 3. Flight Endurance
    # Endurance (hours) = Total Energy (Wh) / Total Power (W)
    # We apply an 80% discharge limit for LiPo batteries to protect them
    usable_energy_factor = 0.8
    endurance_hours = (E_batt_Wh * usable_energy_factor) / P_total
    endurance_minutes = endurance_hours * 60
    
    # --- Plotting the Results ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot 1: Endurance vs Payload Mass
    ax1.plot(m_payload, endurance_minutes, 'b-', linewidth=2.5)
    ax1.set_title('Flight Endurance vs Payload Mass', fontsize=14)
    ax1.set_xlabel('Payload Mass (kg)', fontsize=12)
    ax1.set_ylabel('Endurance (minutes)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # Find a sweet spot: e.g., max payload while maintaining 15 mins of flight
    target_time = 15.0
    if np.any(endurance_minutes >= target_time):
        valid_indices = np.where(endurance_minutes >= target_time)[0]
        optimal_idx = valid_indices[-1] # The heaviest payload that still gives >= 15 min
        opt_payload = m_payload[optimal_idx]
        opt_time = endurance_minutes[optimal_idx]
        ax1.plot(opt_payload, opt_time, 'ro', markersize=8, label=f'Sweet Spot:\n{opt_payload:.2f} kg for {opt_time:.1f} mins')
        ax1.legend()
    
    # Plot 2: Endurance vs Power Consumption
    ax2.plot(P_total, endurance_minutes, 'r-', linewidth=2.5)
    ax2.set_title('Flight Endurance vs Power Consumption', fontsize=14)
    ax2.set_xlabel('Total Power Consumption (W)', fontsize=12)
    ax2.set_ylabel('Endurance (minutes)', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    plot_3d_uav()
    calculate_and_plot_endurance()

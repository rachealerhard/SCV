"""
Analysis of the design space limitations from aerodynamics and batteries

    Nick Goodson
    Feb 2021
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from first_order_range_analysis import gravity, airDensity, dynamicPressure


# ==================
# Unit Conversions
# ==================
kmh_to_ms = 1 / 3.6
Wh_to_J = 3600
s_to_hr = 1 / 3600
m_to_ft = 3.28


def designSpace():
    """
    
    """
    # =========================
    # Definitions
    # =========================
    battery_mass_fraction = 0.3
    propulsive_efficiency = 0.9

    endurance = 1.5 / s_to_hr  # [s]
    altitude = 3200            # [m]
    density = airDensity(altitude)

    # =================================
    # Required L/D for given airspeed
    # =================================
    airspeeds = np.linspace(80, 450, 60)          # [km/h]
    energy_densities = np.linspace(170, 800, 4)   # [Wh/kg]
    LD_required = {}

    for E_star in energy_densities:
        key = str(round(E_star)) + " Wh/kg"
        LD_required[key] = []
        for v in airspeeds:
            LD = (v * kmh_to_ms * endurance * gravity) /\
                 (battery_mass_fraction * propulsive_efficiency * E_star * Wh_to_J)
            LD_required[key].append(LD)
    
    # Visualize
    fig, ax = plt.subplots()
    title = f"Energy Speed Tradeoffs"
    fig.suptitle(title, fontsize=14, fontweight='bold')
    ax.text(260, 40,
            f"Endurance: {endurance * s_to_hr:.1f} hr \
            \nWeight fraction: {battery_mass_fraction * 100:.0f} %\
            \nAltitude: {altitude * m_to_ft:.0f} ft",
            size=18, bbox={'facecolor': 'tab:blue', 'alpha': 0.8, 'pad':6})
    ax.set_xlabel("TAS [km/h]")
    ax.set_ylabel("L/D (aerodynamic efficiency)")
    ax.grid(True)

    for key, values in LD_required.items():
        ax.plot(airspeeds, values, color='tab:blue', linewidth=1.8)
        text_x = airspeeds[len(airspeeds) // 2]
        text_y = values[len(values) // 2]
        ax.text(text_x, text_y, key, size=9, bbox={'facecolor': 'tab:blue', 'alpha': 0.8, 'pad':1})

    # ==================================
    # Aerodynamic Efficiency vs. airspeed
    # ==================================
    vehicles = ["Glider", "Cessna 208", "747-400"]
    drag_coeffs = [0.015, 0.03, 0.031]
    aspect_ratios = [40, 9.7, 7.9]
    wing_areas = [18.61, 25.96, 525]  # [m^2]
    masses = [780, 3629, 400e3]  # [kg]
    e = [0.9, 0.85, 0.8]
    LD_aircraft = {}

    for i, name in enumerate(vehicles):
        LD_aircraft[name] = []
        for v in airspeeds:
            q = dynamicPressure(density, v * kmh_to_ms)
            CL = masses[i] * gravity / (q * wing_areas[i])
            Cd_induced =  (CL ** 2) / (np.pi * aspect_ratios[i] * e[i])
            CD = (drag_coeffs[i] + Cd_induced)
            LD = CL / CD
            LD_aircraft[name].append(LD)

    # Visualize
    colorA = mcolors.CSS4_COLORS['lightcoral']
    colorB = mcolors.CSS4_COLORS['violet']
    colorC = mcolors.CSS4_COLORS['yellowgreen']
    colors = [colorA, colorB, colorC]
    for idx, (key, values) in enumerate(LD_aircraft.items()):
        ax.plot(airspeeds, values, ls='dashed', color=colors[idx])
        text_x = airspeeds[np.argmax(values)-2]
        text_y = np.max(values) + 1
        ax.text(text_x, text_y, key, size=9, bbox={'facecolor': colors[idx], 'alpha': 0.7, 'pad':1})

    plt.show()


def main():
    designSpace()


if __name__ == "__main__":
    main()       

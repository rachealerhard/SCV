"""
First order range analysis for battery-electric transport aircraft

    Nick Goodson
    Jan 2021
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from scipy import integrate

from parametric_study import ParametricAnalysis
from simple_aircraft import Aircraft


# =======================
# Environment Parameters
# =======================
gravity = 9.81  # [m/s^2]
R = 287
density_msl = 1.225  # [kg/m^3]
temp_msl = 288.16  # [K]

# ================
# Unit Conversions
# ================
Wh_to_J = 3600
kmh_to_ms = 1 / 3.6
m_to_km = 1 / 1000


def electricCessna208Analysis():

    # =====================
    # Define Aircraft
    # =====================
    cessna_cfg = { 
            "dry_mass":                2145,           # [kg]
            "battery_mass":            1000,           # [kg]
            "energy_density":          220 * Wh_to_J,     # [J/kg]
            "packing_factor":          0.8,
            "inaccessible_fraction":   0.08,           # fraction of total capacity
            "reserve_fraction":        0.2,            # fraction of useable capacity

            # aero
            "wingspan":                15.87,          # [m]
            "wing_area":               25.96,          # [m^2]
            "wing_efficiency":         0.8,            # Oswald efficiency factor
            "drag_coeff_parasitic":    0.034,           # estimate for small AoA

            # performance
            "cruise_speed":            344 * kmh_to_ms,  # [m/s]
            "cruise_altitude":         3200,             # [m]
            "climb_rate":              6.27,              # [m/s]

            # Propulsion
            "engine_max_power":        503e3,  # [W]
            "prop_efficiency":         0.85,
            "prop_diameter":           2.69  # [m] guesstimate
    }

    cessna208 = Aircraft(cessna_cfg)


    # ======================
    # Fixed Point Analysis
    # ======================
    flight_range = flightRange(cessna208, verbose=True)
    print(f"\nFlight range - electric Cessna 208B: {flight_range / 1000 :.1f} km")


    # =====================
    # Parameteric Analyses
    # =====================
    analysis = ParametricAnalysis(flightRange, cessna208)

    # battery mass
    battery_masses = np.linspace(1000, 2000, 100)
    units = [1, m_to_km]
    battery_ranges = analysis.parametricStudy(["battery_mass"], [battery_masses], display_units=units)

    # # cruise speed
    cruise_speeds = np.linspace(300, 500, 20) * kmh_to_ms
    units = [1 / kmh_to_ms, m_to_km]
    speed_ranges = analysis.parametricStudy(["cruise_speed"], [cruise_speeds], display_units=units)


# =======================
# The Maths
# =======================
def flightRange(cfg, verbose=False):
    """
    Compute the flight range with simple approximations for takeoff
    climb and cruise
    """
    total_weight = cfg.total_mass * gravity  # [N]
    energy_stored = cfg.total_energy * (1 - cfg.inaccessible_fraction) * (1 - cfg.reserve_fraction)  # [J]

    # takeoff
    takeoff_speed = takeoffSpeed(cfg)
    takeoff_energy = takeoffEnergy(takeoff_speed, cfg)
    energy_stored -= takeoff_energy

    # climb
    climb_energy = climbEnergy(takeoff_speed, cfg)
    energy_stored -= climb_energy

    # cruise
    cruise_power = cruisePower(cfg)  # [W]
    flight_range = cfg.cruise_speed * energy_stored / cruise_power

    if verbose:
        print(f"\nTakeoff energy: {takeoff_energy / Wh_to_J :<.2f} [Wh]")
        print(f"Climb energy: {climb_energy / Wh_to_J :<.2f} [Wh]")
        print(f"Cruise power output: {cruise_power / 1000 :<.2f} [kW]")

    return flight_range


def airDensity(altitude):
    """
    Standard atmospheric model (capped at 11 km ~ 35,000 ft)
    """
    if altitude > 11000:
        print("Warning altitude exceeds limit of atmosphere model (11 km)")

    temp_gradient = (216.7 - temp_msl) / 11000
    temperature = altitude * temp_gradient + temp_msl
    density = density_msl * (temperature / temp_msl) ** -(gravity / (temp_gradient * R) + 1)
    return density


def dynamicPressure(density, velocity):
    return 0.5 * density * velocity ** 2


def inducedDrag(dynamic_pressure, cfg):
    """
    Compute the induced drag
    """
    aspect_ratio = (cfg.wingspan ** 2) / cfg.wing_area
    lift_coeff = cfg.total_mass * gravity / (dynamic_pressure * cfg.wing_area)
    drag_coeff_induced =  (lift_coeff ** 2) / (np.pi * aspect_ratio * cfg.wing_efficiency)
    return drag_coeff_induced


def takeoffSpeed(cfg):
    """
    Computes required speed for takeoff from estimate of the lift coefficient
    """
    angle_of_attack = 15 * (np.pi / 180)  # [rad]
    CL0 = 2 * np.pi * angle_of_attack
    aspect_ratio = (cfg.wingspan ** 2) / cfg.wing_area
    lift_coefficient = CL0 / (1 + CL0 / (np.pi * cfg.wing_efficiency * aspect_ratio))
    speed = np.sqrt(cfg.total_mass * gravity / (0.5 * airDensity(1) * lift_coefficient * cfg.wing_area))
    return speed


def takeoffEnergy(takeoff_speed, cfg):
    """
    Estimate the energy usage in takeoff from takeoff distance and engine specs
    Thrust approximation assumes very low free-stream velocity
    """
    density = airDensity(1)
    thrust = (0.5 * density * np.pi * (cfg.engine_max_power * cfg.prop_efficiency * cfg.prop_diameter) ** 2) ** (1 / 3)
    takeoff_time = takeoff_speed * cfg.total_mass / thrust
    takeoff_energy = takeoff_time * cfg.engine_max_power
    return takeoff_energy 


def climbEnergy(takeoff_speed, cfg):
    """
    Estimate energy usage during climb given a known climb rate
    Assumes a linear speed increase up to the cruise speed
    """
    climb_time = cfg.cruise_altitude / cfg.climb_rate

    def climbPower(time):
        speed = (cfg.cruise_speed / climb_time) * time + takeoff_speed  # linear profile
        altitude = cfg.climb_rate * time
        density = airDensity(altitude)
        dynamic_pressure = dynamicPressure(density, speed)
        drag = (cfg.drag_coeff_parasitic + inducedDrag(dynamic_pressure, cfg)) * cfg.wing_area * dynamic_pressure
        thrust = drag + cfg.total_mass * gravity * cfg.climb_rate / speed
        power = thrust * speed
        return power

    climb_energy = integrate.quad(climbPower, 0, climb_time)[0]
    return climb_energy


def cruisePower(cfg):
    """
    Compute power usage during cruise at a specified altitude
    """
    density = airDensity(cfg.cruise_altitude)
    dynamic_pressure = dynamicPressure(density, cfg.cruise_speed)
    drag = cfg.wing_area * dynamic_pressure * (inducedDrag(dynamic_pressure, cfg) + cfg.drag_coeff_parasitic)  # [N]
    cruise_power = drag * cfg.cruise_speed  # [W]
    return cruise_power


def main():
    electricCessna208Analysis()


if __name__ == "__main__":
    main()


# #TODO: Optimize for crusie speed and altitude
# def optimalAltitude(cfg):
#     """
#     Compute the optimal altitude for maximum range at given cruise speed
#     (induced drag = parasitic drag)
#     """
#     lift = cfg.total_mass * gravity
#     optimal_dynamic_pressure = 0.5 * np.sqrt((4 * lift ** 2) / (cfg.wing_area * 
#                             cfg.drag_coeff_parasitic * np.pi * cfg.wing_efficiency * cfg.wingspan ** 2))
#     density = 2 * optimal_dynamic_pressure / (cfg.cruise_speed ** 2)
#     temp_gradient = (216.7 - temp_msl) / 11000
#     temperature = temp_msl * (density / density_msl) ** (-1 / (gravity / (temp_gradient * R) + 1))
#     optimal_altitude = (temperature - temp_msl) / temp_gradient

#     if optimal_altitude > cfg.cruise_ceiling:
#         print(f"Cruise ceiling exceeded: {optimal_altitude / 1000:.2f} [km]")

#     return optimal_altitude




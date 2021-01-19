"""
Range analysis for battery-electric transport aircraft
(based on Cessna 208B Caravan)

    Nick Goodson
    Jan 2021
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import pdb
import argparse


### Environment Parameters ###
gravity = 9.8066  # [m/s^2]
R = 287
density_msl = 1.225  # [kg/m^3]
temp_msl = 288.16  # [K]


### Aircraft Parameters ###
class Cessna208:

    # mass
    turbo_prop_mass = 122.5  # [kg]  P & W PT6A-6
    dry_mass = 2145  # [kg]
    fuel_mass = 1009  # [kg] 
    MTOW = 3629  # [kg] maximum takeoff weight

    # aero
    wingspan = 15.87  # [m]
    wing_area = 25.96 # [m^2]
    wing_efficiency = 0.8  # Oswald efficiency factor
    drag_coeff_parasitic = 0.034  # estimate for small AoA

    # performance
    cruise_speed = 344 / 3.6  # [m/s]
    cruise_ceiling = 7500  # [km]
    climb_rate = 6.27  # [m/s]
    takeoff_distance = 626  # [m]
    standard_range = 1982e3  # [m]

    # Propulsion
    engine_max_power = 503e3  # [W]
    prop_efficiency = 0.85
    prop_diameter = 2.69  # [m] guesstimate

    def __init__(self, cargo_mass=0):
        self.cargo_mass = cargo_mass


class ElectricCessna(Cessna208):

    def __init__(self, cargo_mass=0, battery_mass=None, energy_density=180):
        super().__init__(cargo_mass)
        # propulsion
        self.energy_density = energy_density  # [Wh/kg]
        self.battery_mass = battery_mass if battery_mass else self.fuel_mass  # [kg]

    @property
    def total_mass(self):
        _total_mass = self.dry_mass + self.battery_mass + self.cargo_mass 
        return _total_mass

    @property
    def battery_capacity(self):
        _battery_capacity = self.battery_mass * self.energy_density # [Wh]
        return _battery_capacity


def main(*args):

    aircraft = ElectricCessna()

    n_points = 20
    cargo_masses = np.linspace(10, 1500, n_points)
    battery_masses = np.linspace(200, 1000, n_points)

    flight_ranges = np.zeros((n_points, n_points))
    for i, cargo in enumerate(cargo_masses):
        for j, battery in enumerate(battery_masses):
            aircraft.cargo_mass = cargo
            aircraft.battery_mass = battery
            flight_ranges[i, j] = flightRange(aircraft, verbose=args[0])

    # Visualize results
    range_kms = flight_ranges / 1000
    fig1, ax1 = plt.subplots(figsize=(8,10))
    ax1.contourf(cargo_masses, battery_masses, range_kms)
    ax1.set_ylabel("Cargo mass [kg]")
    ax1.set_xlabel("Battery mass [kg]")
    plt.show()

    fig = plt.figure()
    ax2 = fig.gca(projection='3d')
    CM, BM = np.meshgrid(cargo_masses, battery_masses, indexing='ij') 
    surf = ax2.plot_surface(CM, BM, range_kms, 
                        cmap=cm.coolwarm, linewidth=0, antialiased=False)
    ax2.set_xlabel("Cargo mass [kg]")
    ax2.set_ylabel("Battery mass [kg]")
    ax2.set_zlabel("Range [km]")
    plt.show()


def airDensity(altitude):
    """
    Standard atmospheric model (capped at 11 km ~ 35,000 ft)
    """
    if altitude > 11000:
        raise ValueError("altitude cannot exceed 11 km")
    temp_gradient = (216.7 - temp_msl) / 11000
    temperature = altitude * temp_gradient + temp_msl
    density = density_msl * (temperature / temp_msl) ** -(gravity / (temp_gradient * R) + 1)
    return density


def optimalAltitude(cfg):
    """
    Compute the optimal altitude for maximum range at given cruise speed
    (induced drag = parasitic drag)
    """
    lift = cfg.total_mass * gravity
    optimum_dynamic_pressure = 0.5 * np.sqrt((4 * lift ** 2) / (cfg.wing_area * 
                            cfg.drag_coeff_parasitic * np.pi * cfg.wing_efficiency * cfg.wingspan ** 2))
    density = 2 * optimum_density / (cfg.cruise_speed ** 2)
    temperature = temp_msl * (density / density_msl) ** (-1 / (gravity / (temp_gradient * R) + 1))
    temp_gradient = (216.7 - temp_msl) / 11000
    optimal_altitude = (temperature - temp_msl) / temp_gradient

    if optimal_altitude > cfg.cruise_ceiling:
        print("Cruise ceiling exceeded")
        optimal_altitude = cfg.cruise_ceiling

    return optimal_altitude


def takeoffEnergy(total_weight, cfg):
    """
    Estimate the energy usage in takeoff from takeoff distance and engine specs
    Thrust computation is an approximation assuming a very low free-stream velocity
    """
    # TODO: Replace cfg.takeoff_distance with lift requirement computation.
    density = airDensity(1)
    thrust = (0.5 * density * np.pi * (cfg.engine_max_power * cfg.prop_efficiency * cfg.prop_diameter) **2) ** (1 / 3)
    takeoff_time = np.sqrt(2 * cfg.takeoff_distance * total_weight / thrust)
    takeoff_energy = takeoff_time * cfg.engine_max_power
    return takeoff_energy 


def climbEnergy(cruise_altitude, total_weight, cfg):
    """
    Estimate energy usage during climb given a known climb rate
    """
    climb_time = cruise_altitude / cfg.climb_rate

    # TODO: Implement properly using drag
    climb_energy = total_weight * cruise_altitude * 1.5
    return climb_energy


def flightRange(cfg, verbose=True):
    """
    Compute the flight range with simple approximations for takeoff
    climb and cruise
    """
    total_weight = cfg.total_mass * gravity  # [N]
    aspect_ratio = (cfg.wingspan ** 2) / cfg.wing_area
    energy_stored = cfg.battery_capacity * 3600  # [J]

    # takeoff
    takeoff_energy = takeoffEnergy(cfg)
    energy_stored -= takeoff_energy

    # climb
    cruise_altitude = optimalAltitude(cfg)
    climb_energy = climbEnergy(cruise_altitude, total_weight, cfg)
    energy_stored -= climb_energy

    # cruise
    cruise_altitude = optimalAltitude(cfg) 
    air_density = airDensity(cruise_altitude)
    dynamic_pressure = 0.5 * air_density * cfg.cruise_speed ** 2
    lift_coeff = total_weight / (dynamic_pressure * cfg.wing_area)
    drag_coeff_induced =  (lift_coeff ** 2) / (np.pi * aspect_ratio * cfg.wing_efficiency)
    drag_area_total = cfg.wing_area * (drag_coeff_induced + cfg.drag_coeff_parasitic)
    drag = drag_area_total * dynamic_pressure  # [N]

    cruise_power = drag * cfg.cruise_speed  # [W]
    flight_range = cfg.cruise_speed * energy_stored / cruise_power

    if verbose:
        print(f"\nTakeoff energy: {takeoff_energy / 3600 :<.2f} [Wh]")
        print(f"Climb energy: {climb_energy / 3600 :<.2f} [Wh]")
        print(f"Cruise power output: {cruise_power / 1000 :<.2f} [kW]")

    return flight_range


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action='store_true')
    args = parser.parse_args()
    main(args.verbose)




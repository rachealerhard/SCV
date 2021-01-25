"""
Simulation of an electric Cessna 208B Caravan

    Nick Goodson
    Jan 2021
"""
import argparse
import numpy as np

from basic_range_sim import RangeAnalysis


def main(*args):

    cargo_mass = 300  # [kg]
    aircraft = ElectricCessna(cargo_mass)
    analysis = RangeAnalysis(aircraft, verbose=args[0])

    # battery mass
    battery_masses = np.linspace(50, 1000, 20)
    battery_ranges = analysis.parametricStudy1D("battery_mass", battery_masses)

    # cruise speed
    cruise_speeds = np.linspace(80, 200, 20)
    speed_ranges = analysis.parametricStudy1D("cruise_speed", cruise_speeds)

    # battery mass and cruise speed
    range_tradeoff = analysis.parametricStudy2D("cruise_speed", cruise_speeds,
                                                "battery_mass", battery_masses)
    

### Aircraft ###
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
    cruise_ceiling = 7500  # [m]
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action='store_true')
    args = parser.parse_args()
    main(args.verbose)
    
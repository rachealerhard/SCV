"""
Simple analysis of range for battery-electric transport aircraft (based on Cessna 208 Caravan)
"""
import math
import pdb


### Environment Parameters ###
gravity = 9.8066  # [m/s^2]


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
    wing_efficiency = 0.9  # Oswald efficiency factor
    drag_coeff_parasitic = 0.034  # estimate for small AoA

    # performance
    cruise_speed = 344 / 3.6  # [m/s]
    cruise_altitude = 7500  # [km]
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

    def __init__(self, cargo_mass):
        super().__init__(cargo_mass)
        # propulsion
        self.energy_density = 200  # [Wh/kg]
        self.battery_mass = self.fuel_mass  # [kg]

    @property
    def total_mass(self):
        _total_mass = self.dry_mass + self.battery_mass + self.cargo_mass 
        return _total_mass

    @property
    def battery_capacity(self):
        _battery_capacity = self.battery_mass * self.energy_density # [Wh]
        return _battery_capacity


def compute_air_density(altitude):
    """
    Standard atmospheric model (capped at 11 km ~ 35,000 ft)
    """
    R = 287
    density_msl = 1.225  # [kg/m^3]
    temp_msl = 288.16  # [K]

    if altitude > 11000:
        raise ValueError("altitude cannot exceed 11 km")

    temp_gradient = (216.7 - temp_msl) / 11000
    temp = altitude * temp_gradient + temp_msl
    density = density_msl * (temp / temp_msl) ** -(gravity / (temp_gradient * R) + 1)
    return density


def compute_takeoff_energy(total_weight, cfg):
    """
    Estimate the energy usage in takeoff/climb from the stall speed and engine specs
    Thrust computation is an approximation assuming a very low free-stream velocity
    """
    density = compute_air_density(10)
    thrust = (0.5 * density * math.pi * (cfg.engine_max_power * cfg.prop_efficiency * cfg.prop_diameter) **2) ** (1 / 3)
    takeoff_time = math.sqrt(2 * cfg.takeoff_distance * total_weight / (thrust * gravity))
    climb_time = cfg.cruise_altitude / cfg.climb_rate
    takeoff_energy = (takeoff_time  + climb_time * 0.8) * cfg.engine_max_power
    return takeoff_energy 


def compute_range(cfg, verbose=True):
    """
    Compute the flight range with simple approximations
    """
    total_weight = cfg.total_mass * gravity  # [N]
    aspect_ratio = (cfg.wingspan ** 2) / cfg.wing_area
    energy_stored = cfg.battery_capacity * 3600  # [J]

    # takeoff
    takeoff_energy = compute_takeoff_energy(total_weight, cfg)
    energy_stored -= takeoff_energy

    # cruise
    air_density = compute_air_density(cfg.cruise_altitude)
    dynamic_pressure = 0.5 * air_density * cfg.cruise_speed ** 2
    lift_coeff = total_weight / (dynamic_pressure * cfg.wing_area)
    drag_coeff_induced =  (lift_coeff ** 2) / (math.pi * aspect_ratio * cfg.wing_efficiency)
    drag_area_total = cfg.wing_area * (drag_coeff_induced + cfg.drag_coeff_parasitic)
    drag = drag_area_total * dynamic_pressure  # [N]

    cruise_power = drag * cfg.cruise_speed  # [W]
    flight_range = cfg.cruise_speed * energy_stored / cruise_power

    if verbose:
        print(f"\nTotal mass: {total_weight / gravity :.2f} [kg]")
        print(f"Aspect ratio: {aspect_ratio :.3f}")
        print(f"Induced drag coefficient: {drag_coeff_induced :.3f}")
        print(f"Takeoff energy: {takeoff_energy / 3600 :.2f} [Wh]")
        print(f"Cruise power output: {cruise_power / 1000 :.2f} [kW]")
    return flight_range


def main():

    # Electric
    aircraft = ElectricCessna(cargo_mass=500)
    flight_range = compute_range(aircraft)
    print(f"\nBattery mass: {aircraft.battery_mass :.2f} [kg]")
    print(f"Battery Capacity: {aircraft.battery_capacity :.2f} [Wh]")
    print(f"Cargo mass: {aircraft.cargo_mass :.2f} [kg]")
    print(f"Electric flight range: {flight_range / 1000 :.2f} [km]")


if __name__ == "__main__":
    main()




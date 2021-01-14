"""
Simple analysis of range for battery-electric transport aircraft (based on Cessna 208 Caravan)
"""
import math

### Aircraft Parameters ###
# mass
turbo_prop_mass = 122.5  # [kg]  P & W PT6A-6
dry_mass = 2145  # [kg]
fuel_full_payload = 490  #[kg]
MTOW = 3629  # [kg] maximum takeoff weight

# aero
wingspan = 15.87  # [m]
wing_area = 25.96 # [m^2]
wing_efficiency = 0.95  # Efficiency factor
drag_coefficient_parasitic = 0.034  # estimate for small AoA

# performance
engine_max_power = 503e3  # [W]
prop_efficiency = 0.85
cruise_speed = 344 / 3.6  # [m/s]
cruise_altitude = 7500  # [km]
climb_rate = 6.27  # [m/s]
takeoff_distance = 626  # [m]
standard_range = 1982e3  # [m]

# propulsion
energy_density = 140  # [Wh/kg]
power_output = 200e3  # [W]
battery_mass = 150  # [kg]
prop_diameter = 3.15  # [m] guesstimate


### Environment Parameters ###
gravity = 9.8066  # [m/s^2]



def compute_air_density(altitude):
    """
    Standard atmospheric model (capped at 11 km ~ 35,000 ft)
    """
    R = 0.287
    density_msl = 1.225  # [kg/m^3]
    temp_msl = 288.16  # [K]

    if altitude > 11000:
        raise ValueError("altitude cannot exceed 11 km")

    temp_gradient = (216.7 - temp_msl) / 11000
    temp = altitude * gradient + temp_msl
    density = density_msl * (temp / temp_msl) ** -(gravity / (temp_gradient * R) + 1)
    return density


def compute_takeoff_energy(total_weight):
    """
    Estimate the energy usage in takeoff/climb from the stall speed and engine specs
    Thrust computation is an approximation assuming a very low free-stream velocity
    """
    density = compute_air_density(10)
    thrust = (0.5 * density * math.pi * (engine_max_power * prop_efficiency * prop_diameter) **2) ** (1 / 3)
    takeoff_time = takeoff_speed * total_weight / (thrust * gravity) 
    climb_time = altitude / climb_rate
    takeoff_energy = (takeoff_time  + climb_time * 0.8) * engine_max_power
    return takeoff_energy 


def compute_range(cargo_mass, verbose=True):
    """
    Compute the flight range with simple approximations
    """
    total_weight = (dry_mass + battery_mass + cargo_mass) * gravity  # [N]
    aspect_ratio = (wingspan ** 2) / ref_area
    energy_stored = energy_density * battery_mass  # [Wh]

    # takeoff
    energy_stored -= compute_takeoff_energy(total_weight)

    # cruise
    air_density = compute_air_density(altitude)
    CL = 2 * total_weight / (air_density * ref_area * cruise_speed **2)
    Cd_induced =  (CL ** 2) / (math.pi * aspect_ratio * wing_efficiency)
    Cd_total = Cd_induced

    Drag = pass

    if verbose:
        print(f"aspect ratio: {aspect_ratio}")
        print(f"induced drag coefficient: {Cd_induced}")



def main():
    cargo_mass = MTOW - dry_mass - battery_mass
    pass


if __name__ == "__main__":
    main()




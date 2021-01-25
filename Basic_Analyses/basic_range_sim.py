"""
Range analysis tools for parametric studies of battery-electric transport aircraft

    Nick Goodson
    Jan 2021
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from scipy import integrate
import pdb


### Environment Parameters ###
gravity = 9.8066  # [m/s^2]
R = 287
density_msl = 1.225  # [kg/m^3]
temp_msl = 288.16  # [K]


class RangeAnalysis:

    def __init__(self, aircraft, verbose=False):
        self.aircraft = aircraft
        self.verbose = verbose

    def parametricStudy1D(self, parameter, values):
        """
        Perform a parametric analysis of a single variable taking on specified values
        """
        flight_ranges = np.zeros(len(values))
        for i, val in enumerate(values):
            try:
                setattr(self.aircraft, parameter, val)
            except AttributeError:
                print(f"Error: {parameter1} is not an attribute of {type(self.aircraft).__name__}")
                return
            flight_ranges[i] = flightRange(self.aircraft, verbose=self.verbose)
        self.visualize1D(flight_ranges, parameter, values)
        return flight_ranges

    def parametricStudy2D(self, parameterA, valuesA, parameterB, valuesB):
        """
        Perform a parameteric analysis of two variables taking on specified values
        """
        flight_ranges = np.zeros((len(valuesA), len(valuesB)))
        for i, valA in enumerate(valuesA):
            try:
                setattr(self.aircraft, parameterA, valA)
            except AttributeError:
                print(f"Error: {parameterA} is not an attribute of {type(self.aircraft).__name__}")
                return

            for j, valB in enumerate(valuesB):
                try:
                    setattr(self.aircraft, parameterB, valB)
                except AttributeError:
                    print(f"Error: {parameterB} is not an attribute of {type(self.aircraft).__name__}")
                    return
                flight_ranges[j, i] = flightRange(self.aircraft, verbose=self.verbose)
        self.visualize2D(flight_ranges, parameterA, valuesA, parameterB, valuesB)
        return flight_ranges

    def visualize1D(self, flight_ranges, parameter, values):
        """
        Make a standard plot of 1D parametric sim results
        """
        range_kms = flight_ranges / 1000
        fig, ax = plt.subplots(figsize=(8,10))
        ax.plot(values, range_kms, '-k')
        ax.grid(True)
        ax.set_xlabel(parameter)
        ax.set_ylabel("Range [km]")
        plt.show()

    def visualize2D(self, flight_ranges, parameterA, valuesA, parameterB, valuesB):
        """
        Make some standard plots of 2D parametric sim results
        """
        range_kms = flight_ranges / 1000

        # contours
        fig1, ax1 = plt.subplots(figsize=(8,10))
        CF1 = ax1.contourf(valuesA, valuesB, range_kms)
        ax1.set_xlabel(parameterA)
        ax1.set_ylabel(parameterB)
        cbar = fig1.colorbar(CF1)
        cbar.ax.set_ylabel("Range [km]")

        # 3D surface
        fig2 = plt.figure(figsize=(8,10))
        ax2 = fig2.gca(projection='3d')
        VA, VB = np.meshgrid(valuesA, valuesB, indexing='xy') 
        surf = ax2.plot_surface(VA, VB, range_kms, 
                            cmap=cm.coolwarm, linewidth=0, antialiased=False)
        ax2.set_xlabel(parameterA)
        ax2.set_ylabel(parameterB)
        ax2.set_zlabel("Range [km]")
        plt.show()


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


def optimalAltitude(cfg):
    """
    Compute the optimal altitude for maximum range at given cruise speed
    (induced drag = parasitic drag)
    """
    lift = cfg.total_mass * gravity
    optimal_dynamic_pressure = 0.5 * np.sqrt((4 * lift ** 2) / (cfg.wing_area * 
                            cfg.drag_coeff_parasitic * np.pi * cfg.wing_efficiency * cfg.wingspan ** 2))
    density = 2 * optimal_dynamic_pressure / (cfg.cruise_speed ** 2)
    temp_gradient = (216.7 - temp_msl) / 11000
    temperature = temp_msl * (density / density_msl) ** (-1 / (gravity / (temp_gradient * R) + 1))
    optimal_altitude = (temperature - temp_msl) / temp_gradient

    if optimal_altitude > cfg.cruise_ceiling:
        print(f"Cruise ceiling exceeded: {optimal_altitude / 1000:.2f} [km]")
        # optimal_altitude = cfg.cruise_ceiling
    return optimal_altitude


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
    thrust = (0.5 * density * np.pi * (cfg.engine_max_power * cfg.prop_efficiency * cfg.prop_diameter) **2) ** (1 / 3)
    takeoff_time = takeoff_speed * cfg.total_mass / thrust
    takeoff_energy = takeoff_time * cfg.engine_max_power
    return takeoff_energy 


def climbEnergy(cruise_altitude, takeoff_speed, cfg):
    """
    Estimate energy usage during climb given a known climb rate
    Assumes a linear speed increase up to the cruise speed
    """
    climb_time = cruise_altitude / cfg.climb_rate

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


def cruisePower(cruise_altitude, cfg):
    """
    Compute power usage during cruise at a specified altitude
    """
    density = airDensity(cruise_altitude)
    dynamic_pressure = dynamicPressure(density, cfg.cruise_speed)
    drag = cfg.wing_area * dynamic_pressure * (inducedDrag(dynamic_pressure, cfg) + cfg.drag_coeff_parasitic)  # [N]
    cruise_power = drag * cfg.cruise_speed  # [W]
    return cruise_power


def flightRange(cfg, verbose=True):
    """
    Compute the flight range with simple approximations for takeoff
    climb and cruise
    """
    total_weight = cfg.total_mass * gravity  # [N]
    energy_stored = cfg.battery_capacity * 3600  # [J]

    # takeoff
    takeoff_speed = takeoffSpeed(cfg)
    takeoff_energy = takeoffEnergy(takeoff_speed, cfg)
    energy_stored -= takeoff_energy

    # climb
    cruise_altitude = optimalAltitude(cfg)
    climb_energy = climbEnergy(cruise_altitude, takeoff_speed, cfg)
    energy_stored -= climb_energy

    # cruise
    cruise_power = cruisePower(cruise_altitude, cfg)  # [W]
    flight_range = cfg.cruise_speed * energy_stored / cruise_power
    print(flight_range)

    if verbose:
        print(f"\nTakeoff energy: {takeoff_energy / 3600 :<.2f} [Wh]")
        print(f"Climb energy: {climb_energy / 3600 :<.2f} [Wh]")
        print(f"Cruise power output: {cruise_power / 1000 :<.2f} [kW]")

    return flight_range







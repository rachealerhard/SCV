"""
Zeroeth order range analysis for battery electric aircraft
    - Uses electric range equation and generic parameters

    Nick Goodson
    Feb 2021
"""
import numpy as np
import pdb

from parametric_study import ParametricAnalysis
from simple_aircraft import Aircraft


# =======================
# Environment Parameters
# =======================
gravity = 9.81  # [m/s^2]

# ======================
# Unit Conversions
# ======================
Wh_to_J = 3600
kmh_to_ms = 1 / 3.6
m_to_km = 1 / 1000


def zeroOrderRangeAnalysis():

    # =====================
    # Define Aircraft
    # =====================
    aircraft_cfg = { 
            "dry_mass":                3000,           # [kg]
            "battery_mass":            1000,           # [kg]
            "energy_density":          220 * Wh_to_J,  # [J/kg]
            "packing_factor":          0.8,
            "capcity_fade":            0.15,           # fraction of total capacity
            "inaccessible_fraction":   0.08,           # fraction of total capacity
            "reserve_fraction":        0.2,            # fraction of useable capacity
            "cruise_fraction":         0.6,            # fraction of useable capacity
            "drivetrain_efficiency":   0.85,           # Includes prop
            "LD_ratio":                11
    }

    aircraft = Aircraft(aircraft_cfg)


    # ======================
    # Fixed Point Analysis
    # ======================
    performance_new = electricRangeEqn(aircraft)
    performance_eol = electricRangeEqn(aircraft, end_of_life=True)
    print(f"\nCruise range (new): {performance_new['cruise_range'] / 1000 :.1f} km")
    print(f"Cruise range (end of life): {performance_eol['cruise_range'] / 1000:.1f} km")


    # =====================
    # Parameteric Analyses
    # =====================
    analysis = ParametricAnalysis(cruiseRange, aircraft)

    # L/D ratio
    LD_s = np.arange(8, 40, 1)
    units = [1, m_to_km]
    ranges = analysis.parametricStudy(['LD_ratio'], [LD_s], display_units=units)

    # L/D ratio and energy density
    E_densities = np.arange(220, 800, 10) * Wh_to_J  # [J/kg]
    params = ['LD_ratio', 'energy_density']
    vals = [LD_s, E_densities]
    units = [1, 1 / Wh_to_J, m_to_km]
    ranges = analysis.parametricStudy(params, vals, display_units=units, end_of_life=True)

    # battery mass and dry mass
    battery_masses = np.arange(500, 2000, 20)
    dry_masses = np.arange(1600, 6000, 200)
    params = ['battery_mass', 'dry_mass']
    vals = [battery_masses, dry_masses]
    units = [1, 1, m_to_km]
    analysis.parametricStudy(params, vals, display_units=units, end_of_life=False)


# ========================
# The Maths
# ========================
def electricRangeEqn(aircraft, end_of_life=False):
    """
    Computes aircraft cruise range using the 'electric range formula'
    
    @param aircaft, instance of Aircraft defining basic parameters
    @param end_of_life (bool): when true, battery capacity fade is included
    @return outputs (dict): battery_mass_fraction
                            total_energy
                            useable_energy
                            reserve_energy
                            cruise_energy
                            cruise_range
    """
    outputs = {}
    outputs['battery_mass_fraction'] = aircraft.battery_mass / aircraft.total_mass
    total_energy = aircraft.total_energy
    if end_of_life:
        total_energy *= (1 - aircraft.capcity_fade)
    outputs['useable_energy'] = total_energy * (1 - aircraft.inaccessible_fraction)
    outputs['reserve_energy'] = outputs['useable_energy'] * aircraft.reserve_fraction
    outputs['cruise_energy'] = outputs['useable_energy'] * aircraft.cruise_fraction
    outputs['cruise_range'] = aircraft.drivetrain_efficiency * aircraft.LD_ratio *\
                                outputs['cruise_energy'] / (gravity * aircraft.total_mass)

    return outputs


def cruiseRange(aircraft, end_of_life=False):
    """
    A wrapper around electricCruiseRange for use in parametric
    analyses
    """
    outputs = electricRangeEqn(aircraft, end_of_life=end_of_life)
    return outputs['cruise_range']


def main():
    """

    """
    zeroOrderRangeAnalysis()


if __name__ == "__main__":
    main()

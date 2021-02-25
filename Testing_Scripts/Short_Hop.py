# Short_Hop.py
#
# Created: Feb 2021, N. Goodson
# Modified:

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units

import time
import numpy as np

import sys
# sys.path.append('../Vehicle_Optimization')
# import Plot_Mission

sys.path.append('../Vehicles')
from Cessna_208B_electric import vehicle_setup

sys.path.append('../Missions')
from short_hop_30min_cruise_reserve import mission as mission_setup

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    """
    Mission:    A short flight designed to be repeated 3 times on a single charge while
                maintaining the required 30 min battery reserve
    Output:     Maximum range of each hop
    """
    cases = np.array([1])
    for case in cases:
        
        if case == 1:
            cargo_mass = 2300 * Units.lb
            battery_mass = 2361 * Units.kg
            
        elif case ==2:
            cargo_mass = 1300 * Units.lb
            battery_mass = 2361 * Units.kg + 1000 * Units.lb
            
        elif case ==3:
            battery_mass = 1590 * Units.kg
            combined_mass = 3404.262451
            cargo_mass = combined_mass - battery_mass
            
        elif case ==4:
            cargo_mass = 2300 * Units.lb
            battery_mass = 1009 * Units.kg

        print(f"\n---------------------------------------\n"
              f"Case {case } \n"
              f"---------------------------------------\n")
        start_t = time.time()    
        results = payload_range_analysis(cargo_mass, battery_mass)
        elapsed = (time.time() - start_t)/60 
        print(f"\nElapsed time: {elapsed :.2f} [min] ")
        
        # Plot_Mission.plot_mission(results)
    
    return


#-------------------------------------------------------------------------------
# Simulation Function
#-------------------------------------------------------------------------------

def payload_range_analysis(cargo_mass, battery_mass):

    vehicle  = vehicle_setup(cargo_mass, battery_mass)

    # Analysis
    analyses = base_analysis(vehicle)
    mission  = mission_setup(analyses, vehicle)
    
    analyses.mission = mission
    analyses.finalize()
    
    results = mission.evaluate()
    analyze_results(vehicle, results)

    return results


def analyze_results(vehicle, results):
    base = vehicle
    res  = results.segments
    
    # Extract total aircraft weight
    total_weight = base.mass_properties.max_takeoff
    
    # Check total range:
    mission_range = res[-1].conditions.frames.inertial.position_vector[-1,0]    
    mission_time = res[-1].conditions.frames.inertial.time[-1,0]
    reserve_flight_range = 144.841  # 30 min (0.5 hr) reserve at 180 mph
    range_w_reserve = (mission_range / 1000) - reserve_flight_range

    # Final Energy
    maxcharge         = base.propulsors.battery_propeller.battery.max_energy
    extra_energy      = res[-1].conditions.propulsion.battery_energy[-1,0] #(maxcharge - res[-1].conditions.propulsion.battery_energy[-1,0])
    battery_remaining = extra_energy/maxcharge
    reserve_energy = res.cruise_reserve.conditions.propulsion.battery_energy[0,0] - res.cruise_reserve.conditions.propulsion.battery_energy[-1,0]
    energy_usage       = (maxcharge-extra_energy-reserve_energy)
    
    print(f"\nBattery weight: {base.mass_properties.battery_mass :.6f} [kg] ")
    print(f"Empty weight: {base.mass_properties.operating_empty :.6f} [kg]")
    print(f"Payload weight: {base.mass_properties.max_payload :.6f} [kg]")
    print(f"Total weight: {total_weight :.6f} [kg]\n") 
    
    print(f"Battery remaining: {battery_remaining :.6f} ")
    print(f"Energy usage: {energy_usage/Units.kWh :.6f} [kWh]")
    print(f"Total energy: {maxcharge/Units.kWh :.6f} [kWh] \n")
        
    print(f"Range: {range_w_reserve :.6f} [km] \n")
    print(f"Mission time: {mission_time / 60:.6f} [min]")

    return 

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights = SUAVE.Analyses.Weights.Weights_Transport() #Weights_UAV()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    return analyses


if __name__ == '__main__':
    main()
# payload_range_tradeoff.py
#
# Created: Feb 2021, R. Erhard
# Modified:

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

import SUAVE

from SUAVE.Core import Units, Data
from SUAVE.Methods.Performance.electric_payload_range import electric_payload_range
from SUAVE.Plots.Geometry_Plots.plot_vehicle import plot_vehicle  


import time
import numpy as np
import matplotlib.pyplot as plt
import Plot_Mission
import sys
sys.path.append('../Vehicles')
from DEP_Aircraft import vehicle_setup

sys.path.append('../Missions')
from full_mission_30min_cruise_reserve_variable_cruise import mission as mission_setup

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    '''
    Base case:   2300lb payload, 530km range + 30min cruise reserve
    Output:      battery mass required and resulting energy usage
   
    '''
    cases = np.array([1]) #, 2, 3, 4.1, 4.2, 4.3, 4.4])
    for case in cases:
        
        if case == 1:
            cargo_mass = 2300 * Units.lb
            battery_mass = 2311 * Units.kg
            base_vehicle = 'extra_bat'
            
        elif case ==2:
            cargo_mass = 1300 * Units.lb
            battery_mass = 2311 * Units.kg + 1000 * Units.lb
            base_vehicle = 'extra_bat'
            
        elif case ==3:
            battery_mass = 1590 * Units.kg
            combined_mass = 3404.262451
            cargo_mass = combined_mass - battery_mass
            base_vehicle = 'extra_bat'
        elif case ==0:
            battery_mass = 3404.262451
            combined_mass = 3404.262451
            cargo_mass = combined_mass - battery_mass
            base_vehicle = 'extra_bat'            
            
        elif case ==4.1:
            cargo_mass = 2300 * Units.lb
            battery_mass = 1009 * Units.kg
            base_vehicle = 'converted'
            
        elif case ==4.2:
            cargo_mass = 2000 * Units.lb
            battery_mass = 1009 * Units.kg
            base_vehicle = 'converted'
            
        elif case ==4.3:
            cargo_mass = 1000 * Units.lb
            battery_mass = 1009 * Units.kg   
            base_vehicle = 'converted'
            
        elif case == 4.4:
            cargo_mass = 0. * Units.lb
            battery_mass = 1009 * Units.kg   
            base_vehicle = 'converted'
            
        print(f"\n---------------------------------------\n"
              f"Case {case } \n"
              f"---------------------------------------\n")
        start_t = time.time()    
        results = payload_range_analysis(cargo_mass, battery_mass, base_vehicle)
        elapsed = (time.time() - start_t)/60 
        print(f"\nElapsed time: {elapsed :.2f} [min] ")
        
        Plot_Mission.plot_mission(results)
    
    return


#-------------------------------------------------------------------------------
# Test Function
#-------------------------------------------------------------------------------

def payload_range_analysis(cargo_mass, battery_mass, base_vehicle):

    vehicle  = vehicle_setup(cargo_mass,battery_mass)
    print(f"\nEmpty weight: {vehicle.mass_properties.operating_empty :.2f} [kg] ")

    # Analysis
    analyses = base_analysis(vehicle)
    mission  = mission_setup(analyses,vehicle)
    
    analyses.mission = mission
    analyses.finalize()
    
    results = mission.evaluate()
    analyze_results(vehicle,results)

    return results


def analyze_results(vehicle,results):
    base = vehicle
    res  = results.segments
    
    # Extract total aircraft weight
    total_weight = base.mass_properties.max_takeoff
    
    # Check total range:
    mission_range = res[-1].conditions.frames.inertial.position_vector[-1,0]    
    mission_time = res[-1].conditions.frames.inertial.time[-1,0]
    range_w_reserve = (mission_range/1000) - 144.841 # 30min reserve at 180mph
    
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
        
    
    return 

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    return configs


def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

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
    #aerodynamics.settings.use_surrogate              = False
    #aerodynamics.settings.propeller_wake_model       = True     
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
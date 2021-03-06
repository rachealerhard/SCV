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

import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../Vehicles')
#from Cessna_208B_electric import vehicle_setup

from DEP_Aircraft import vehicle_setup
sys.path.append('../Missions')
from full_mission_30min_cruise_reserve_variable_cruise import mission as mission_setup

from cruise_mission_fixed_mission_profile import mission as mission_setup

#-------------------------------------------------------------------------------
# Test Function
#-------------------------------------------------------------------------------

def main():
    cargo_mass = 1000 * Units.lb
    battery_mass = 1009 * Units.kg  
    
    vehicle  = vehicle_setup(cargo_mass,battery_mass)
    analyses = base_analysis(vehicle)
    mission  = mission_setup(analyses,vehicle)
    
    analyses.mission = mission
    analyses.finalize()
    
    results = mission.evaluate()
    analyze_results(vehicle,results)
    
    plot_vehicle(vehicle, save_figure = False, plot_control_points = False)
    plt.show()    
        
    
    stop_flag = 1
    
    #===============
    
    #payload_range = electric_payload_range(vehicle, mission, 'cruise', display_plot=False)
    ## Vehicle analyses
    #configs_analyses  = analyses_setup(configs)
    #missions          = full_mission_setup(vehicle, configs_analyses) 
    
    #analyses = SUAVE.Analyses.Analysis.Container()
    #analyses.missions = missions
    
    #configs.finalize()
    #analyses.finalize()
    
    #mission = analyses.missions.mission
    #results = mission.evaluate()
    
    #plot_vehicle(configs.base, save_figure = False, plot_control_points = False)
    #plt.show()    
    
    #payload_range = electric_payload_range(vehicle, missions.mission, 'cruise', display_plot=False)
    
    ## Plot the payload-range diagram:
    #R   = payload_range.range
    #PLD = payload_range.payload
    #TOW = payload_range.takeoff_weight

    #plt.plot(R/1000, PLD, 'r', label='Battery Mass: ' + str(battery_mass) +'kg')
    #plt.xlabel('Range (km)')
    #plt.ylabel('Payload (kg)')
    #plt.title('Payload Range Diagram\n(Fixed Battery Weight)')
    #plt.grid(True)
    #plt.legend()
    #plt.show()    

    return


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
    aerodynamics.settings.use_surrogate              = False
    aerodynamics.settings.propeller_wake_model       = True     
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

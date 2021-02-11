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

import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../Vehicles')
from Cessna_208B_electric import vehicle_setup

sys.path.append('../Missions')
from cruise_mission import cruise_mission_setup
from full_mission_variable_cruise   import full_mission_setup
#-------------------------------------------------------------------------------
# Test Function
#-------------------------------------------------------------------------------

def main():

    vehicle     = vehicle_setup()
    configs     = configs_setup(vehicle)
    analyses    = analyses_setup(configs)
    mission     = full_mission_setup(vehicle, analyses) 

    analyses.mission = mission
    analyses.finalize()
    
    payload_range = electric_payload_range(vehicle, mission, 'cruise', display_plot=False)
    
    # Plot the payload-range diagram:
    R   = payload_range.range
    PLD = payload_range.payload
    TOW = payload_range.takeoff_weight

    plt.plot(R/1000, PLD, 'r')
    plt.xlabel('Range (km)')
    plt.ylabel('Payload (kg)')
    plt.title('Payload Range Diagram\n(Fixed Battery Weight)')
    plt.grid(True)
    plt.show()    

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

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.wings['main_wing'].control_surfaces.flap.deflection = 0. * Units.deg
    configs.append(config)


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg

    configs.append(config)


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection = 30. * Units.deg

    configs.append(config)


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

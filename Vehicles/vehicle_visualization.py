# vehicle_visualization.py
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

#from Cessna_208B_electric import vehicle_setup
from DEP_Aircraft import vehicle_setup

import sys
sys.path.append('../Missions')
from cruise_mission_fixed_mission_profile import mission as mission_setup

#-------------------------------------------------------------------------------
# Test Function
#-------------------------------------------------------------------------------

def main():
    # Set vehicle mass properties and geometry
    cargo_mass = 1000 * Units.lb
    battery_mass = 1009 * Units.kg
    vehicle  = vehicle_setup(cargo_mass,battery_mass)
    
    # Set wake visualization
    wake_visualization = False
    
    if wake_visualization:
        
        analyses = base_analysis(vehicle)
        mission  = mission_setup(analyses,vehicle)
        
        analyses.mission = mission
        analyses.finalize()
        
        results = mission.evaluate()
    
    plot_vehicle(vehicle, save_figure = False, plot_control_points = False)
    plt.show() 
    
    return
    
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

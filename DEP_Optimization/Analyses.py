# Analyses.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE

# ----------------------------------------------------------------------        
#   Setup Analyses
# ----------------------------------------------------------------------  

def setup(configs):
    
    analyses = SUAVE.Analyses.Analysis.Container()
    
    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base(config)
        analyses[tag] = analysis
    
    return analyses

# ----------------------------------------------------------------------        
#   Define Base Analysis
# ----------------------------------------------------------------------  

def base(vehicle):
    
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
    #aerodynamics.settings.propeller_wake_model = True
    #aerodynamics.settings.number_spanwise_vortices  = 10 
    #aerodynamics.settings.number_chordwise_vortices = 2    
    analyses.append(aerodynamics)
    
    # ------------------------------------------------------------------
    #  Stability Analysis
    # ------------------------------------------------------------------
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)    
    
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
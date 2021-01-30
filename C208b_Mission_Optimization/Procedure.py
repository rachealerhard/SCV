# Procedure.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

from SUAVE.Core import Units
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
from SUAVE.Plots.Mission_Plots import *

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing   = simple_sizing
    
    # Size the battery and charge it before the mission
    procedure.weights_battery = weights_battery

    # finalizes the data dependencies
    procedure.finalize        = finalize
    
    # performance studies
    procedure.missions        = Process()
    procedure.missions.base   = simple_mission
    
    # Post process the results
    procedure.post_process    = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Simple Mission
# ----------------------------------------------------------------------    
    
def simple_mission(nexus):
    
    mission = nexus.missions.mission

    # Evaluate the missions and save to results   
    results         = nexus.results
    results.mission = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    
    # Pull out the vehicle
    base = nexus.vehicle_configurations.base
    
    # wing spans,areas, and chords
    for wing in base.wings:
        
        # Set all of the areas for the surfaces
        wing.areas.wetted   = 1.75 * wing.areas.reference
        wing.areas.exposed  = 0.8  * wing.areas.wetted
        wing.areas.affected = 0.6  * wing.areas.wetted   

 
    # diff the new data
    base.store_diff()

    return nexus

# ----------------------------------------------------------------------
#   Calculate weights and charge the battery
# ---------------------------------------------------------------------- 

def weights_battery(nexus):

    # Evaluate weights for all of the configurations
    config = nexus.analyses.base
    config.weights.evaluate() 
    
    base     = nexus.vehicle_configurations.base
    payload = base.mass_properties.cargo 
    MTOW    = base.mass_properties.max_takeoff
    empty   = base.weight_breakdown.empty
    
    # Calculate battery mass
    batmass = MTOW - empty - payload  #-mmotor
    bat     = base.propulsors.battery_propeller.battery
    initialize_from_mass(bat,batmass)
    base.propulsors.battery_propeller.battery.mass_properties.mass = batmass
        
    # Set Battery Charge
    maxcharge = nexus.vehicle_configurations.base.propulsors.battery_propeller.battery.max_energy
    nexus.missions.mission.segments[0].battery_energy = maxcharge 

    return nexus
    
# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus         

# ----------------------------------------------------------------------
#   Post Process results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack
    mis  = nexus.missions.mission.segments.cruise
    base = nexus.vehicle_configurations.base
    res  = nexus.results.mission.segments
    
    # Check total range:
    mission_range = res[-1].conditions.frames.inertial.position_vector[-1,0]    
       
    # Final Energy
    maxcharge    = base.propulsors.battery_propeller.battery.max_energy
    extra_energy = res[-1].conditions.propulsion.battery_energy[-1,0] #(maxcharge - res[-1].conditions.propulsion.battery_energy[-1,0])
    
    battery_remaining = extra_energy/maxcharge
    # Energy constraints, the battery doesn't go to zero anywhere, using a P norm
    p                    = 8.    
    energies             = res[-1].conditions.propulsion.battery_energy[:,0]/np.abs(maxcharge)
    energies[energies>0] = 0.0 # Exclude the values greater than zero
    energy_constraint    = np.sum((np.abs(energies)**p))**(1/p) 
    
    # Pack up
    summary = nexus.summary
    summary.base_mission_range    = -mission_range          # maximize range
    summary.battery_remaining     = 100*(battery_remaining - 0.3) # leaves 30% reserve
    
    return nexus    


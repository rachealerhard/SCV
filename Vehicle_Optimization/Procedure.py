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
    
    # compute the vehicle weights, size and charge the battery
    procedure.weights_battery = weights_battery

    # finalize the data dependencies
    procedure.finalize        = finalize
    
    # performance studies
    procedure.missions        = Process()
    procedure.missions.base   = simple_mission
    
    # post-process the results
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
    
    empty   = base.weight_breakdown.empty # structural weight; function of vehicle geometry
    payload = base.mass_properties.max_payload
    batmass = base.mass_properties.battery_mass
    
    MTOW    = empty + payload + batmass #base.weight_breakdown.max_takeoff
    base.mass_properties.max_takeoff = MTOW
    
    # update the battery parameters based on the battery mass
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
    
    # Extract total aircraft weight
    total_weight = base.mass_properties.max_takeoff
    
    # Check total range:
    mission_range = res[-1].conditions.frames.inertial.position_vector[-1,0]    
    mission_time = res[-1].conditions.frames.inertial.time[-1,0]
       
    # Final Energy
    maxcharge         = base.propulsors.battery_propeller.battery.max_energy
    extra_energy      = res[-1].conditions.propulsion.battery_energy[-1,0] #(maxcharge - res[-1].conditions.propulsion.battery_energy[-1,0])
    battery_remaining = extra_energy/maxcharge

    # Pack up
    summary = nexus.summary
    #summary.mission_time       = mission_time
    summary.mission_range       = mission_range   
    summary.nothing            = 0.0
    summary.battery_remaining  = battery_remaining
    summary.payload            = base.mass_properties.max_payload
    #summary.energy_usage       = (maxcharge-extra_energy)
    summary.total_weight       = total_weight
    
    return nexus    


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
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv, size_optimal_motor
from SUAVE.Plots.Mission_Plots import *

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    procedure = Process()
    
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
#   Calculate weights and charge the battery
# ---------------------------------------------------------------------- 

def weights_battery(nexus):

    # Evaluate weights for all of the configurations
    config = nexus.analyses.base
    base     = nexus.vehicle_configurations.base
    
    # update the battery parameters based on the new battery mass
    bat     = base.propulsors.battery_propeller.battery
    initialize_from_mass(bat,batmass)
    base.propulsors.battery_propeller.battery = bat #.mass_properties.mass = batmass
    base.propulsors.battery_propeller.voltage = bat.max_voltage
    
    # update the motor based on the new battery parameters
    propeller_motor   = base.propulsors.battery_propeller.motor
    propeller_motor.nominal_voltage      = bat.max_voltage 
    propeller_motor                      = size_optimal_motor(propeller_motor, base.propulsors.battery_propeller.propeller)
    base.propulsors.battery_propeller.motor      = propeller_motor
    
    # converge on weights (since landing gear weight is a function of MTOW)
    converged = False
    prior_empty = 0
    while converged ==False:
        config.weights.evaluate()         
        base     = nexus.vehicle_configurations.base
        
        empty   = base.weight_breakdown.empty # structural weight; function of vehicle geometry
        payload = base.mass_properties.max_payload
        batmass = base.mass_properties.battery_mass
        
        MTOW    = empty + payload + batmass #base.weight_breakdown.max_takeoff
        for segment_config in nexus.vehicle_configurations:
            segment_config.mass_properties.max_takeoff = MTOW
            segment_config.mass_properties.takeoff = MTOW
            segment_config.mass_properties.operating_empty = empty
        if abs(empty-prior_empty) <1e-5:
            converged = True
        else:
            prior_empty = empty        
    
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
    range_w_reserve = (mission_range) - 144841. # 30min reserve at 180mph
    mission_time = res[-1].conditions.frames.inertial.time[-1,0]
       
    # Final Energy
    maxcharge         = base.propulsors.battery_propeller.battery.max_energy
    extra_energy      = res[-1].conditions.propulsion.battery_energy[-1,0] # (maxcharge - res[-1].conditions.propulsion.battery_energy[-1,0])
    battery_remaining_after_reserve = extra_energy/maxcharge
    
    reserve_energy = res.cruise_reserve.conditions.propulsion.battery_energy[0,0] - res.cruise_reserve.conditions.propulsion.battery_energy[-1,0]
    energy_usage       = (maxcharge-abs(extra_energy)-abs(reserve_energy))    
    battery_remaining = (extra_energy+reserve_energy)/maxcharge    
    
    # Pack up
    summary = nexus.summary
    #summary.mission_time       = mission_time
    summary.mission_range       = range_w_reserve   
    summary.nothing            = 0.0
    summary.battery_remaining_after_reserve  = battery_remaining_after_reserve
    summary.battery_remaining  = battery_remaining
    summary.payload            = base.mass_properties.max_payload
    summary.energy_usage       = energy_usage
    summary.total_weight       = total_weight
    summary.payload            = base.mass_properties.max_payload
    
    range_required = 530 * Units.km
    summary.specified_range = abs(range_required - range_w_reserve)
    
   
    print(f"\nBattery weight: {base.mass_properties.battery_mass :.6f} [kg] ")
    print(f"Empty weight: {base.mass_properties.operating_empty :.6f} [kg]")
    print(f"Payload weight: {base.mass_properties.max_payload :.6f} [kg]")
    print(f"Total weight: {total_weight :.6f} [kg]\n") 
    
    print(f"Battery remaining (after reserve): {battery_remaining_after_reserve :.6f} ")
    print(f"Battery remaining (before reserve): {battery_remaining :.6f} ")
    print(f"Energy usage: {summary.energy_usage/Units.kWh :.6f} [kWh]")
    print(f"Maxcharge: {maxcharge/Units.kWh :.6f} [kWh] \n")
    

    print(f"Required Range: {range_required/1000 :.6f} [km]")    
    print(f"Mission Range: {range_w_reserve/1000 :.6f} [km] \n")    
    

    return nexus    


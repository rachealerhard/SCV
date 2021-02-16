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
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing   = simple_sizing
    
    # compute the vehicle weights, size and charge the battery
    procedure.weights_battery = weights_battery
    
    ## spin propellers before aerodynamic analysis to capture DEP interaction
    #procedure.propeller_spin  = spin_props    

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

    ## Change the dynamic pressure based on the, add a factor of safety   
    #base.envelope.maximum_dynamic_pressure = nexus.missions.mission.segments.cruise.dynamic_pressure*1.2

    # Scale the horizontal and vertical tails based on the main wing area
    base.wings.horizontal_stabilizer.areas.reference = 0.15 * base.reference_area
    base.wings.vertical_stabilizer.areas.reference   = 0.08 * base.reference_area

    # wing spans,areas, and chords
    for wing in base.wings:

        # Unpack
        AR = wing.aspect_ratio
        S  = wing.areas.reference
        taper_ratio = wing.taper

        # Set the spans
        wing.spans.projected = np.sqrt(AR*S)

        # Set all of the areas for the surfaces
        wing.areas.wetted   = 1.75* S
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted   

        # Set all of the chord lengths
        chord = wing.areas.reference/wing.spans.projected
        wing.chords.mean_aerodynamic = chord
        wing.chords.mean_geometric   = chord
        wing.chords.tip              = 2*chord/(1+(1/taper_ratio))
        wing.chords.root             = wing.chords.tip/taper_ratio
        

    # Resize the motor
    motor = base.propulsors.battery_propeller.motor
    prop = base.propulsors.battery_propeller.propeller
    motor = size_optimal_motor(motor,prop)    

    # diff the new data
    base.store_diff()

    return nexus

# ----------------------------------------------------------------------
#   Calculate weights and charge the battery
# ---------------------------------------------------------------------- 

def weights_battery(nexus):

    # Evaluate weights for all of the configurations
    config = nexus.analyses.base
    
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
    
    ## update the battery parameters based on the battery mass
    #bat     = base.propulsors.battery_propeller.battery
    #initialize_from_mass(bat,batmass)
    #base.propulsors.battery_propeller.battery.mass_properties.mass = batmass
    
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

def spin_props(nexus):
    propeller = nexus.vehicle_configurations.base.propulsors.battery_propeller
    conditions = nexus.results.mission.segments.base.state.conditions
    F, Q, P, Cp, outputs , etap = propeller.spin(conditions)
    propeller.outputs = outputs
    nexus.vehicle_configurations.base.propulsors.battery_propeller = propeller
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
    
    # Aerodynamics in cruise
    cruise_aoa = res.cruise.state.conditions.aerodynamics.angle_of_attack[0,0] / Units.deg
    parasitic_drag = res.cruise.state.conditions.aerodynamics.drag_breakdown.parasite.total[0,0]
    induced_drag   = res.cruise.state.conditions.aerodynamics.drag_breakdown.induced.total[0,0]
    total_drag     = res.cruise.state.conditions.aerodynamics.drag_breakdown.total[0][0]
    trim_corrected_drag = res.cruise.state.conditions.aerodynamics.drag_breakdown.trim_corrected_drag[0][0]
    
    # Propeller swept area
    base.propulsors.battery_propeller.propeller
    
    
    # Pack up
    summary = nexus.summary
    #summary.mission_time       = mission_time
    summary.mission_range       = mission_range   
    summary.nothing            = 0.0
    summary.battery_remaining  = battery_remaining
    summary.payload            = base.mass_properties.max_payload
    summary.energy_usage       = (maxcharge-extra_energy)
    summary.total_weight       = total_weight
    summary.objective          = (summary.total_weight/1e3) + (summary.energy_usage/1e8)
    
   
    print(f"\nBattery weight: {base.mass_properties.battery_mass :.6f} [kg] ")
    print(f"Empty weight: {base.mass_properties.operating_empty :.6f} [kg]")
    print(f"Payload weight: {base.mass_properties.max_payload :.6f} [kg]")
    print(f"Total weight: {total_weight :.6f} [kg]\n") 
    
    print(f"Battery remaining: {battery_remaining :.6f} ")
    print(f"Energy usage: {summary.energy_usage/Units.kWh :.6f} [kWh]")
    print(f"Maxcharge: {maxcharge/Units.kWh :.6f} [kWh] \n")
    
    print(f"Cruise angle of attack: {cruise_aoa :.6f} [deg]")
    print(f"CD cruise: {total_drag :.6f} [-]")
    print(f"CD induced: {induced_drag :.6f} [-]")
    print(f"CD parasitic: {parasitic_drag :.6f} [-]")    
    print(f"CD trim: {trim_corrected_drag :.6f} [-]\n")      
    
    return nexus    


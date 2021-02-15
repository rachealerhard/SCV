# cruise_mission_fixed_mission_profile.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units
import numpy as np

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
'''
Defines a fixed cruise segment profile

'''
    
def cruise_mission_setup(vehicle,analyses):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container() 
    
    missions.mission = mission(analyses,vehicle)

    return missions  
    
def mission(analyses,vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    

    ones_row     = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = 4
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_propeller.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.battery_propeller.residuals
    
    base_segment.state.unknowns.propeller_power_coefficient  = 0.16 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)             
    #base_segment.battery_energy           = vehicle.base.propulsors.battery_propeller.battery.max_energy


    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses )

    segment.altitude  = 3500. * Units.meter 
    segment.air_speed = 180.  * Units.mph
    segment.distance  = 100 * Units.kilometer #412.37  * Units.kilometer
    segment.state.unknowns.throttle   = 0.9 *  ones_row(1)  
    segment.battery_energy           = vehicle.propulsors.battery_propeller.battery.max_energy
    
    # add to misison
    mission.append_segment(segment)    
    
    
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

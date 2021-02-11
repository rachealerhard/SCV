# cruise_mission.py
#
# Created: Feb 2021, R. Erhard
# Modified:

import SUAVE
from SUAVE.Core import Units, Data

def cruise_mission_setup(vehicle, analyses):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Variable_Range_Cruise.Given_State_of_Charge()
    mission.tag = 'cruise_mission'
    
    mission.cruise_tag = 'cruise'
    mission.target_state_of_charge = 0.3

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.state.numerics.number_control_points        = 4
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    

    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_propeller.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.battery_propeller.residuals
    base_segment.state.unknowns.propeller_power_coefficient  = 0.16 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)             
    base_segment.battery_energy                              = vehicle.propulsors.battery_propeller.battery.max_energy

    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 2500. * Units.meter 
    segment.air_speed = 180.  * Units.mph
    segment.distance  = 50.   * Units.kilometer 
    segment.state.unknowns.throttle = 0.8 *  ones_row(1)  
    segment.battery_energy          = vehicle.propulsors.battery_propeller.battery.max_energy
    
    # add to misison
    mission.append_segment(segment)    
    
    
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------


    return mission
# full_mission_fixed_mission_profile.py
#
# Created: Feb 2021, R. Erhard
# Modified:

import SUAVE
from SUAVE.Core import Units, Data

def full_mission_setup(vehicle,analyses):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container() 
    
    missions.mission = mission(analyses,vehicle)

    return missions  
def mission(analyses, vehicle):
    ''' 
    This sets up a full mission with a fixed sequential mission profile. 
    Will be useful for optimizing for minimum mission time of a fixed 
    pre-determined mission.
    
    '''
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Variable_Range_Cruise.Given_State_of_Charge()
    mission.tag = 'mission'
    
    mission.cruise_tag = 'cruise'
    mission.target_state_of_charge = 0.15 # 10% nonusable after 30min reserve    
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.state.numerics.number_control_points        = 4

    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_propeller.unpack_unknowns    
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    

    base_segment.process.iterate.residuals.network           = vehicle.propulsors.battery_propeller.residuals
    base_segment.state.unknowns.propeller_power_coefficient  = 0.16 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)             
    
    # ------------------------------------------------------------------
    #   First Climb Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses )

    segment.altitude_start = 0.   * Units.meter
    segment.altitude_end   = 2000. * Units.meter
    segment.air_speed      = 125.  * Units.mph
    segment.climb_rate     = 1000.  * Units['ft/min'] # max climb rate for the Cessna Caravan is 1234 ft/min
    segment.battery_energy           = vehicle.propulsors.battery_propeller.battery.max_energy
    segment.state.unknowns.throttle  = 0.85 * ones_row(1)  
    
    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses )

    segment.altitude_start = 2000. * Units.meter
    segment.altitude_end   = 3500. * Units.meter
    segment.air_speed      = 160.  * Units.mph
    segment.climb_rate     = 1000. * Units['ft/min'] 
    segment.state.unknowns.throttle  = 0.85 * ones_row(1)  
    
    # add to misison
    mission.append_segment(segment)
     

    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses )

    segment.altitude  = 3500. * Units.meter 
    segment.air_speed = 180.  * Units.mph
    segment.distance  = 10.  * Units.kilometer 
    segment.state.unknowns.throttle = 0.8 *  ones_row(1)
    
    # add to misison
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #   Second Cruise Segment: 30 minutes of reserve
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_reserve"

    segment.analyses.extend( analyses )

    segment.altitude  = 3500. * Units.meter 
    segment.air_speed = 180.  * Units.mph
    segment.distance  = segment.air_speed * 30*Units.minutes
    segment.state.unknowns.throttle = 0.8 *  ones_row(1)
    
    # add to misison
    mission.append_segment(segment)        
    
    # ------------------------------------------------------------------
    #   Descent Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent" 
    segment.analyses.extend( analyses ) 
    segment.altitude_start            = 3500. * Units.meter
    segment.altitude_end              = 2000.  * Units.meter
    segment.air_speed                 = 150. * Units['mph']  
    segment.climb_rate                = -500.  * Units['ft/min']  
    segment.state.unknowns.throttle   = 0.9 * ones_row(1)  
    
    # add to misison
    mission.append_segment(segment)     
    

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------


    return mission
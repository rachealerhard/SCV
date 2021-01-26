# Cessna_208B.py
# 
# Created:  Jan 2021, R. Erhard

""" optimizing cruise distance and speed
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Plots.Mission_Plots import *
from scipy.optimize import minimize

import numpy as np
import pylab as plt

# Use converted Cessna 208B Electric Aircraft:
import sys
sys.path.append('.')
sys.path.append('../Vehicle_Scripts')
from vehicle_setup_Cessna_208B_electric import vehicle_setup




# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
    # setup
    vehicle, configs, analyses = full_setup(cruise_distance=75*Units.km, cruise_speed=190*Units.mph)
    simple_sizing(configs)
    configs.finalize()
    analyses.finalize()

    # run full mission    
    mission = analyses.missions.base
    results = mission.evaluate()
    
    # remove energy used for takeoff and landing segments
    cruise_starting_energy, max_cruise_energy = available_cruise_energy(results)
    
    # optimize cruise segment : find speed and distance to maximize range while staying within available cruise energy
 
    # Optimize cruise distance to give 20% battery reserve after mission: 
    x_opt, v_opt, range_opt = opt_cruise(vehicle,cruise_starting_energy,max_cruise_energy)
    
    
    
    ## plt the old results
    #plot_mission(results)


    return

def opt_cruise(vehicle,cruise_starting_energy,available_energy):
    # find cruise distance and speed that optimizes range and gives 20% battery reserve
    # bounds of variables:   
    x_lb = 40. * Units.kilometer 
    x_ub = 250. * Units.kilometer
    
    #v_lb = 150. * Units.mph
    #v_ub = 215. * Units.mph
    
    arguments = (vehicle,cruise_starting_energy,available_energy)
    cons = [{'type':'ineq', 'fun': bat_reserve, 'args': arguments}]
    
    bnds = [(x_lb,x_ub)]
    tolerance = 1e-6
    sol = minimize(evaluate_cruise_mission, [70 * Units.kilometer],args=arguments, method='SLSQP', bounds=bnds, constraints=cons, tol=tolerance)
    
    # optimized result
    x_opt = sol.x[0] 
    #v_opt = sol.x[1] * v_factor
    range_opt = sol.fun
    
    return x_opt, range_opt

def bat_reserve(cruise_params,vehicle,cruise_starting_energy,available_energy):
 
    cruise_distance = cruise_params[0]
    #cruise_speed    = cruise_params[1] * v_factor
    
    
    # run cruise mission
    configs, analyses = cruise_setup(vehicle, available_energy, cruise_distance, cruise_speed=190*Units.mph)
    simple_sizing(configs)
    configs.finalize()
    analyses.finalize()
    mission = analyses.missions.base
    results = mission.evaluate()       

    # Check battery energy at end of mission:
    spent_cruise_energy = results.segments[0].conditions.propulsion.battery_energy[0,0]  - results.segments[0].conditions.propulsion.battery_energy[-1,0] 
    remaining_energy = (available_energy - spent_cruise_energy)

    return remaining_energy

def evaluate_cruise_mission(cruise_params,vehicle,cruise_starting_energy,available_energy):
    
    cruise_distance = cruise_params[0] 
    #cruise_speed    = cruise_params[1] * v_factor
    
    # run cruise mission
    configs, analyses = cruise_setup(vehicle, available_energy, cruise_distance, cruise_speed=190*Units.mph)
    simple_sizing(configs)
    configs.finalize()
    analyses.finalize()
    mission = analyses.missions.base
    results = mission.evaluate()       

    # Check battery energy used:
    spent_cruise_energy = results.segments[0].conditions.propulsion.battery_energy[0,0]  - results.segments[0].conditions.propulsion.battery_energy[-1,0] 
    remaining_energy = (available_energy - spent_cruise_energy)
    
    # Check total range:
    mission_range = results.segments[-1].conditions.frames.inertial.position_vector[-1,0]
    
    # Check takeoff weight:
    takeoff_weight = results.segments[0].conditions.weights.total_mass[0,0] # [kg]
    
    obj_val = mission_range
    return - obj_val # maximize range


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(cruise_distance,cruise_speed):
    vehicle = vehicle_setup()
    configs  = configs_setup(vehicle)
    
    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses, vehicle,cruise_distance, cruise_speed)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return vehicle, configs, analyses



def cruise_setup(vehicle,available_energy,cruise_distance,cruise_speed):
    cruise_configs  = cruise_configs_setup(vehicle)
    
    # vehicle analyses
    cruise_config_analyses = analyses_setup(cruise_configs)

    # mission analyses
    mission  = cruise_mission_setup(cruise_config_analyses,vehicle,available_energy,cruise_distance, cruise_speed)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = cruise_config_analyses
    analyses.missions = missions_analyses

    return cruise_configs, analyses


    
def available_cruise_energy(results):
    climb1_energy = results.segments[0].conditions.propulsion.battery_energy[0,0]  - results.segments[0].conditions.propulsion.battery_energy[-1,0] 
    climb2_energy = results.segments[1].conditions.propulsion.battery_energy[0,0]  - results.segments[0].conditions.propulsion.battery_energy[-1,0] 
    descent_energy = results.segments[-1].conditions.propulsion.battery_energy[0,0] - results.segments[-1].conditions.propulsion.battery_energy[-1,0] 
    
    takeoff_energy_used = climb1_energy + climb2_energy
    landing_energy_used = descent_energy
    non_usable_energy = 0.1 * results.segments[0].conditions.propulsion.battery_energy[0,0]
    
    cruise_start_energy = results.segments[0].conditions.propulsion.battery_energy[0,0] - takeoff_energy_used
    available_energy = cruise_start_energy - (landing_energy_used+non_usable_energy)
    return cruise_start_energy, available_energy    

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

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
    weights = SUAVE.Analyses.Weights.Weights_Transport()  
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()    
    #aerodynamics.process.compute.lift.inviscid.settings.spanwise_vortex_density    = 3
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()    
    #stability.settings.spanwise_vortex_density                  = 3
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses    


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def cruise_configs_setup(vehicle):
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
    return configs
    

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


# ----------------------------------------------------------------------
#   Sizing for the Vehicle Configs
# ----------------------------------------------------------------------

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 1.75 * wing.areas.reference
        wing.areas.exposed  = 0.8  * wing.areas.wetted
        wing.areas.affected = 0.6  * wing.areas.wetted


    # diff the new data
    base.store_diff()

    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle, cruise_distance,cruise_speed):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

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
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)             
    
    # ------------------------------------------------------------------
    #   First Climb Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 0.   * Units.meter
    segment.altitude_end   = 2000. * Units.meter
    segment.air_speed      = 125.  * Units.mph
    segment.climb_rate     = 1000.  * Units['ft/min'] # max climb rate for the Cessna Caravan is 1234 ft/min
    segment.battery_energy           = vehicle.propulsors.battery_propeller.battery.max_energy * 0.89
    segment.state.unknowns.throttle  = 0.85 * ones_row(1)  
    
    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 2000. * Units.meter
    segment.altitude_end   = 4500. * Units.meter
    segment.air_speed      = 150.  * Units.mph
    segment.climb_rate     = 1000. * Units['ft/min'] # max climb rate for the Cessna Caravan
    segment.state.unknowns.throttle  = 0.85 * ones_row(1)  
    
    # add to misison
    mission.append_segment(segment)
     

    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 4500. * Units.meter # 7620.  * Units.meter
    segment.air_speed = cruise_speed # 180.   * Units.mph
    segment.distance  = cruise_distance #120.   * Units.kilometer # honolulu to kauai is about 160 km 
    segment.state.unknowns.throttle   = 0.9 *  ones_row(1)  

    # add to misison
    mission.append_segment(segment)    
    
    
    # ------------------------------------------------------------------
    #   Descent Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "decent" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_start            = 4500. * Units.meter # 7620.  * Units.meter
    segment.altitude_end              = 2500  * Units.meter
    segment.air_speed                 = 140. * Units['mph']  
    segment.climb_rate                = - 500.  * Units['ft/min']  
    segment.state.unknowns.throttle   = 0.9 * ones_row(1)  
    
    # add to misison
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def cruise_mission_setup(analyses,vehicle,available_energy, cruise_distance,cruise_speed):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

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
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)             
    
    

    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 4500. * Units.meter # 7620.  * Units.meter
    segment.air_speed = cruise_speed # 180.   * Units.mph
    segment.distance  = cruise_distance #120.   * Units.kilometer # honolulu to kauai is about 160 km 
    segment.battery_energy           = available_energy
    segment.state.unknowns.throttle   = 0.9 *  ones_row(1)  

    # add to misison
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission



def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions  

def plot_mission(results,line_style='bo-'):

    axis_font = {'fontname':'Arial', 'size':'14'}  
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Drag Components
    plot_drag_components(results, line_style)    
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)
    
    # Plot Aircraft Electronics
    plot_electronic_conditions(results, line_style)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)
    
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)       
        
    return



def set_axes(axes):
    """This sets the axis parameters for all plots
    """       
    axes.minorticks_on()
    axes.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axes.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')      
    axes.grid(True)   
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)        

    return  

if __name__ == '__main__': 
    main()    
    plt.show()
    e = 1.
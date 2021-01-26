# Cessna_208B.py
# 
# Created:  Jan 2021, R. Erhard

""" Evaluate the effect of increasing vehicle weight (by adding batteries) on range for fixed cargo load
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Plots.Mission_Plots import *


# Use converted Cessna 208B Electric Aircraft:
from vehicle_setup_Cessna_208B_electric import vehicle_setup
from scipy.optimize import minimize

import numpy as np
import pylab as plt



# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # For a fixed cargo load, evaluate the mtow/range tradeoff
    cargo_mass         = 443. * Units.kilogram # fixed
    structural_mass    = 2533. * Units.kilogram # eventually will scale with increased battery weight after sizing up the wing for the given MTOW
    base_battery_mass  = 400 *Units.kilogram #1009. * Units.kilogram
    extra_battery_mass = np.linspace(0.0, 700.0 * Units.kilogram, 8)
    
    # Set specific energy (pre-packing)
    e_bat = 450. * Units.Wh/Units.kg
    
    battery_mass = base_battery_mass + extra_battery_mass
    MTOW = structural_mass + battery_mass + cargo_mass
    
    range_w_reserve = np.zeros_like(MTOW)
    
    for i in range(len(MTOW)):
        takeoff_mass = MTOW[i]
        bat_mass     = battery_mass[i]       
    
        range_w_reserve[i] = compute_max_range(takeoff_mass,bat_mass,cargo_mass,e_bat)
    
    # Plot results:

    fig = plt.figure()
    axes = fig.add_subplot(1, 1, 1)
    axes.plot(range_w_reserve/Units.kilometer, MTOW, "o-", color='tab:red', label="Takeoff Mass") 
    axes.plot(172,3985,label='Direct Convert')
    axes.set_xlabel("Range [km]")
    axes.set_ylabel("Takeoff Weight [kg]")
    axes.set_title("Takeoff Weight vs Range Diagram")
    plt.legend()
    plt.grid()
    plt.show()

    ## plt the old results
    #plot_mission(results)


    return

def compute_max_range(takeoff_mass,battery_mass, payload,e_bat):
    # setup vehicle and configs
    configs, analyses = full_setup(takeoff_mass,battery_mass, payload,e_bat)
    simple_sizing(configs)
    configs.finalize()
    analyses.finalize()      
    
    # optimize for range by adjusting cruise distance
    cruise_distance, max_range = cruise_opt(configs, analyses)
    
    return max_range

def cruise_opt(configs,analyses):
    
    x_factor = 1e5
    
    # set bounds on cruise distance
    x_lb = 3. * Units.kilometer / x_factor
    x_ub = 600. * Units.kilometer / x_factor
    
    arguments = (configs,analyses)
    cons = [{'type':'ineq', 'fun': bat_reserve, 'args':arguments}]
    
    bnds = [(x_lb,x_ub)]
    tolerance = 1e-8
    sol = minimize(obj_fun, [10. * Units.kilometer / x_factor], method='SLSQP', bounds=bnds, constraints=cons, args=arguments, tol=tolerance,options={'disp': True})
    
    # optimized result
    x_opt = sol.x * x_factor
    range_opt = -sol.fun * x_factor
    extra_energy = bat_reserve(sol.x,configs,analyses) * x_factor
    
    print(f"\nMaximum range: {range_opt/1000} [km]")
    print(f"Cruise distance: {x_opt[0]/1000} [km]")
    print(f"Remaining energy: {extra_energy} [Wh]")
    #print(f"Battery mass: {vehicle.propulsors.battery_propeller.battery.mass_properties.mass} [kg]")
    return x_opt, range_opt

def obj_fun(x_distance,configs,analyses):
    x_factor = 1e5
    x_distance = x_distance[0] * x_factor
    
    # set the mission and adjust cruise distance
    mission = analyses.missions.base
    mission.segments.cruise.distance = x_distance # first make sure accessing it properly, it should start out as 70 km
    
    # analyze the mission
    results = mission.evaluate()
    total_range = results.segments[-1].conditions.frames.inertial.position_vector[-1,0]
    
    return -total_range/x_factor

def bat_reserve(x_distance,configs,analyses):
    x_factor = 1e5
    x_distance = x_distance[0] * x_factor
    
    # set the mission and adjust cruise distance
    mission = analyses.missions.base
    mission.segments.cruise.distance = x_distance # first make sure accessing it properly, it should start out as 70 km
    
    # analyze the mission
    results = mission.evaluate()

    # Check battery energy at end of mission:
    starting_energy  = results.segments[0].conditions.propulsion.battery_energy[0,0] # not including 11% non-usable energy
    remaining_energy = results.segments[-1].conditions.propulsion.battery_energy[-1,0]
    
    required_reserve = 0.1 * starting_energy
    extra_energy   = (remaining_energy - required_reserve)/Units.Wh # >= 0
    if extra_energy < 0.:
        print(f"\nConstraint not satisfied, x_distance: {x_distance/1000} [km]")
        print(f"Extra energy: {extra_energy} [Wh]")

    return extra_energy/x_factor
# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(takeoff_mass, battery_mass, payload, e_bat):

    # vehicle data
    vehicle = vehicle_setup(takeoff_mass, battery_mass, payload, e_bat) 
    
    configs  = configs_setup(vehicle)

    # initialize vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    
    # adjust cruise distance
    #
    
    missions_analyses = missions_setup(mission)
    
    # initialize the mission analyses to be performed

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

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

def mission_setup(analyses,vehicle):

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
    segment.air_speed = 180.   * Units.mph
    segment.distance  = 70.   * Units.kilometer # honolulu to kauai is about 160 km 
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
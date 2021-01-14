# Cessna_208B.py
# 
# Created:  Jan 2021, R. Erhard

""" setup file for Cessna 208B Super Cargomaster Aircraft
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Plots.Mission_Plots import *

import numpy as np
import pylab as plt


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # Setup the vehicle and analyses
    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()  

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)


    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)


    return analyses
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

    # done!
    return analyses    


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Cessna_208B'    


    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff   = 8750. * Units.pounds
    vehicle.mass_properties.takeoff       = 8750. * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 8750. * Units.pounds
    vehicle.mass_properties.cargo         = 0. 

    # envelope properties
    vehicle.envelope.ultimate_load = 5.7 # Change
    vehicle.envelope.limit_load    = 3.8 # Change

    # basic parameters
    vehicle.reference_area         = 279. * Units.feet**2       
    vehicle.passengers             = 9

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 279. * Units.feet**2
    wing.spans.projected         = 52.  * Units.feet + 1. * Units.inches

    wing.chords.root             = 6. * Units.feet + 6. * Units.inches 
    wing.chords.tip              = 4.2136 * Units.feet 
    wing.chords.mean_aerodynamic = 5.3568 * Units.feet 
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference # 9.702

    wing.twists.root             = 3.0 * Units.degrees # Change
    wing.twists.tip              = 1.5 * Units.degrees # Change

    wing.origin                  = [[130.* Units.inches,0,0]]
    wing.aerodynamic_center      = [17.* Units.inches,0,0]

    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.95
    wing.areas.reference         = 5800. * Units.inches**2 # Change
    wing.spans.projected         = 136.  * Units.inches # Change

    wing.chords.root             = 55. * Units.inches # Change
    wing.chords.tip              = 30. * Units.inches # Change
    wing.chords.mean_aerodynamic = 43. * Units.inches # Guess
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[385.* Units.inches,0,0]]
    wing.aerodynamic_center      = [15.* Units.inches,0,0] # Change
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = False

    wing.dynamic_pressure_ratio  = 0.9

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    

    wing.sweeps.quarter_chord    = 25. * Units.deg # Change
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 3500. * Units.inches**2 # Change
    wing.spans.projected         = 73.   * Units.inches # Change

    wing.chords.root             = 66. * Units.inches # Change
    wing.chords.tip              = 27. * Units.inches # Change
    wing.chords.mean_aerodynamic = 48. * Units.inches # Guess
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[385.* Units.inches,0,  0.832]]
    wing.aerodynamic_center      = [20.* Units.inches,0,0]  # Change

    wing.vertical                = True 
    wing.symmetric               = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.seats_abreast         = 2.

    fuselage.fineness.nose         = 1.6
    fuselage.fineness.tail         = 2.

    fuselage.lengths.nose          = 82.  * Units.inches
    fuselage.lengths.tail          = 205. * Units.inches
    fuselage.lengths.cabin         = 164. * Units.inches
    fuselage.lengths.total         = 451. * Units.inches
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.    

    fuselage.width                 = 64. * Units.inches

    fuselage.heights.maximum       = 84. * Units.inches # Change
    fuselage.heights.at_quarter_length          = 62. * Units.inches # Change
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches # Change
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches # Change

    fuselage.areas.side_projected  = 8000.  * Units.inches**2. # Change
    fuselage.areas.wetted          = 30000. * Units.inches**2. # Change
    fuselage.areas.front_projected = 42.* 62. * Units.inches**2. # Change

    fuselage.effective_diameter    = 70. * Units.inches


    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------ 
    
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    landing_gear.main_tire_diameter = 0.5587 * Units.m
    landing_gear.nose_tire_diameter = 0.381 * Units.m
    landing_gear.main_strut_length = 0.721 * Units.m
    landing_gear.nose_strut_length = 0.839 * Units.m
    landing_gear.main_units = 2     #number of main landing gear units
    landing_gear.nose_units = 1     #number of nose landing gear
    landing_gear.main_wheels = 1    #number of wheels on the main landing gear
    landing_gear.nose_wheels = 1    #number of wheels on the nose landing gear      
    vehicle.landing_gear=landing_gear    

  
    # ------------------------------------------------------------------
    #   Piston Propeller Network
    # ------------------------------------------------------------------    
    
    # build network
    net = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag = 'network'
    net.number_of_engines = 1.
    net.nacelle_diameter  = 42 * Units.inches
    net.engine_length     = 0.01 * Units.inches
    net.areas             = Data()
    net.rated_speed       = 2700. * Units.rpm
    net.areas.wetted      = 0.01

    # Component 1 the engine
    net.engine = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    net.engine.sea_level_power    = 503. * Units.kilowatts
    net.engine.flat_rate_altitude = 0.0
    net.engine.speed              = 2700. * Units.rpm
    net.engine.BSFC               = 0.52


    # Design the Propeller    
    prop_attributes = Data()
    prop_attributes.number_blades       = 3.0
    prop_attributes.freestream_velocity = 119.   * Units.knots  # Change
    prop_attributes.angular_velocity    = 2100.  * Units.rpm    # Change
    prop_attributes.tip_radius          = 106./2. * Units.inches 
    prop_attributes.hub_radius          = 11.1    * Units.inches 
    prop_attributes.design_Cl           = 0.8 
    prop_attributes.design_altitude     = 12000. * Units.feet
    prop_attributes.design_thrust       = None #0.0
    prop_attributes.design_power        = .64 * 503 * Units.kilowatts

    prop_attributes.airfoil_geometry       = ["NACA_23012.txt"]
    prop_attributes.airfoil_polars         = [["naca23012_50000.txt", "naca23012_100000.txt", "naca23012_200000.txt", "naca23012_500000.txt", "naca23012_1000000.txt"]]  
    prop_attributes.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] #np.zeros(20)
    prop_attributes.rotation = [-1]
    prop_attributes                     = propeller_design(prop_attributes)    

    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.prop_attributes = prop_attributes
    net.propeller        = prop

    # add the network to the vehicle
    vehicle.append_component(net)  
 
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle


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

    configs.append(config)


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    #config.wings['main_wing'].flaps.angle = 0. * Units.deg

    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2.
    
    config.landing_gear.gear_condition = 'down'

    configs.append(config)


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    #config.wings['main_wing'].flaps_angle = 30. * Units.deg

    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.

    configs.append(config)


    return configs

# ----------------------------------------------------------------------
#   Sizing for the Vehicle Configs
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    axis_font = {'fontname':'Arial', 'size':'14'}  
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)    

        
    return

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

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # diff the new data
    landing.store_diff()

    # done!
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

    ##airport
    #airport = SUAVE.Attributes.Airports.Airport()
    #airport.altitude   =  0.0  * Units.ft
    #airport.delta_isa  =  0.0
    #airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    #mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    #base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.network.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.network.residuals     
    
    ## ------------------------------------------------------------------
    ##   First Climb Segment: constant Speed, constant rate segment 
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    #segment.tag = "climb_1"

    #segment.analyses.extend( analyses.takeoff )

    #segment.altitude_start = 0.0   * Units.km
    #segment.altitude_end   = 1000. * Units.feet
    #segment.air_speed      = 73.5  * Units['knots']
    #segment.climb_rate     = 695.  * Units['ft/min']
    
    ## add to misison
    #mission.append_segment(segment)
   
    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 8000. * Units.feet
    segment.air_speed = 92.   * Units['knots']
    segment.distance  = 100.   * Units.nautical_mile

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



if __name__ == '__main__': 
    main()    
    plt.show()
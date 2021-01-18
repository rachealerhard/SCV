# vehicle_setup_Cessna208B.py
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
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars import (
    compute_airfoil_polars,
)

import numpy as np
import pylab as plt



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
    vehicle.mass_properties.max_takeoff     = 3985. * Units.kilogram
    vehicle.mass_properties.takeoff         = 3985. * Units.kilogram
    vehicle.mass_properties.operating_empty = 2533. * Units.kilogram
    vehicle.mass_properties.max_zero_fuel   = 2533. * Units.kilogram
    vehicle.mass_properties.cargo           = 1451. * Units.kilogram 

    # envelope properties
    vehicle.envelope.limit_load    = 3.8 # airplane designed to withstand 3.8 positive g's before deformation begins
    vehicle.envelope.ultimate_load = 5.7 # limit load with a safety factor of 1.5
    
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
    
    # Main wing control surface:
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.20 
    flap.span_fraction_end     = 0.70   
    flap.deflection            = 0.0 * Units.degrees
    flap.configuration_type    = 'double_slotted'
    flap.chord_fraction        = 0.30   
    wing.append_control_surface(flap)       
    

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
    wing.areas.reference         = 6.51 * Units.meter**2 
    wing.spans.projected         = 6.25 * Units.meter

    wing.chords.root             = 1.225 * Units.meter 
    wing.chords.tip              = 0.858 * Units.meter 
    wing.chords.mean_aerodynamic = 1.0416 * Units.meter 
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
    wing.areas.reference         = 3.57 * Units.meter**2 	
    wing.spans.projected         = 2.05 *Units.meter 

    wing.chords.root             = 2.049 * Units.meter
    wing.chords.tip              = 1.434 * Units.meter
    wing.chords.mean_aerodynamic = 1.741 * Units.meter
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

    fuselage.effective_diameter    = 80. * Units.inches


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
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_blades       = 3.0
    prop.freestream_velocity = 185.   * Units.knots
    prop.angular_velocity    = 2100.  * Units.rpm    # Change
    prop.tip_radius          = 106./2. * Units.inches 
    prop.hub_radius          = 11.1    * Units.inches 
    prop.design_Cl           = 0.8 
    prop.design_altitude     = 12000. * Units.feet
    prop.design_thrust       = None #0.0
    prop.design_power        = .64 * 503 * Units.kilowatts

    #prop_attributes.airfoil_geometry       = ["NACA_23012.txt"]
    #prop_attributes.airfoil_polars         = [["naca23012_50000.txt", "naca23012_100000.txt", "naca23012_200000.txt", "naca23012_500000.txt", "naca23012_1000000.txt"]] 
    prop.airfoil_geometry = ["Clark_y.txt"]
    prop.airfoil_polars = [[
            "Clark_y_polar_Re_100000.txt",
            "Clark_y_polar_Re_200000.txt",
            "Clark_y_polar_Re_500000.txt",
            "Clark_y_polar_Re_1000000.txt",
        ]]    
    polar_stations = np.zeros(20)
    prop.airfoil_polar_stations = list(polar_stations.astype(int))
    prop.rotation = [-1]
    prop                     = propeller_design(prop)  
    
    airfoil_polars = compute_airfoil_polars(prop, prop.airfoil_geometry, prop.airfoil_polars)
    airfoil_cl_surs = airfoil_polars.lift_coefficient_surrogates
    airfoil_cd_surs = airfoil_polars.drag_coefficient_surrogates
    prop.airfoil_cl_surrogates = airfoil_cl_surs
    prop.airfoil_cd_surrogates = airfoil_cd_surs
    
    net.propeller        = prop

    # add the network to the vehicle
    vehicle.append_component(net)  
 
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

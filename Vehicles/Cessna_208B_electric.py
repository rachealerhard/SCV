# vehicle_setup_Cessna208B.py
# 
# Created:  Jan 2021, R. Erhard

""" setup file for Cessna Caravan
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

from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass

from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
#from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
from SUAVE.Methods.Propulsion.electric_motor_sizing            import size_optimal_motor
import numpy as np
import pylab as plt
import os



# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Cessna_208B_electric'    
    vehicle.systems.accessories = 'short-range'
    vehicle.systems.control = 'partially powered'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff     = 3985. * Units.kilogram  * 1.1
    vehicle.mass_properties.takeoff         = 3985. * Units.kilogram  * 1.1
    vehicle.mass_properties.battery_mass    = 1009 * Units.kg
    vehicle.mass_properties.max_payload     = 1500. * Units.lb
    
    #vehicle.mass_properties.operating_empty = vehicle.mass_properties.max_takeoff - vehicle.mass_properties.max_payload 

    
    # envelope properties
    vehicle.envelope.limit_load    = 3.8 # airplane designed to withstand 3.8 positive g's before deformation begins
    vehicle.envelope.ultimate_load = 5.7 # limit load with a safety factor of 1.5
    
    # basic parameters
    vehicle.reference_area         = 279. * Units.feet**2       # will be set from constraint diagram analysis
    vehicle.passengers             = 9

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing     = main_wing_inputs(wing) # inputs to main wing to be optimized 
    
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
    
    wing = hstab_inputs(wing)

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

    wing = vstab_inputs(wing)
    
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

    fuselage = fuselage_inputs(fuselage)

    # add to vehicle
    vehicle.append_component(fuselage)
    

    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    
    # build network    
    net = Battery_Propeller()
    net.number_of_engines = 1. # run various discrete cases
    net.nacelle_diameter  = 42 * Units.inches
    net.engine_length     = 0.01 * Units.inches
    
    
    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc
    
    
    # Component 2 the Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades       = 3.0
    prop.freestream_velocity = 180.   * Units.mph
    prop.angular_velocity    = 1700.  * Units.rpm   
    prop.design_altitude     = 12000. * Units.feet
    prop.design_thrust       = None #0.0
    prop.design_power        = .64 * 503 * Units.kilowatts    
    
    prop = prop_inputs(prop)
    
    airfoils_path = os.path.join(os.path.dirname(__file__), "Polars/")
    prop.airfoil_geometry = [airfoils_path + "Clark_y.txt"]
    prop.airfoil_polars = [[
            airfoils_path + "Clark_y_polar_Re_100000.txt",
            airfoils_path + "Clark_y_polar_Re_200000.txt",
            airfoils_path + "Clark_y_polar_Re_500000.txt",
            airfoils_path + "Clark_y_polar_Re_1000000.txt",
        ]]    
    polar_stations = np.zeros(20)
    prop.airfoil_polar_stations = list(polar_stations.astype(int))
    
    prop.symmetry = True
    prop          = propeller_design(prop)  # optimizes the propeller blade twist, thickness, etc.
    
    airfoil_polars = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)
    airfoil_cl_surs = airfoil_polars.lift_coefficient_surrogates
    airfoil_cd_surs = airfoil_polars.drag_coefficient_surrogates
    prop.airfoil_cl_surrogates = airfoil_cl_surs
    prop.airfoil_cd_surrogates = airfoil_cd_surs
    
    net.propeller        = prop
    
    
    # Component: Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    #bat = bat_inputs(bat)
    bat.mass_properties.mass = 1009 * Units.kg
    bat.specific_energy      = (450 *Units.Wh/Units.kg) * 0.8 # weighted by packing factor
    bat.resistance           = 0.006
    bat.max_voltage          = 500.
    
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery       = bat
    net.voltage       = bat.max_voltage
        

    # Propeller  motor
    propeller_motor                      = SUAVE.Components.Energy.Converters.Motor()
    propeller_motor.efficiency           = 0.95
    propeller_motor.nominal_voltage      = bat.max_voltage 
    propeller_motor.mass_properties.mass = 10.0  * Units.kg
    propeller_motor.origin               = prop.origin  
    propeller_motor.propeller_radius     = prop.tip_radius      
    propeller_motor.no_load_current      = 2.0  
    propeller_motor                      = size_optimal_motor(propeller_motor,prop)
    net.motor                            = propeller_motor
    

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 10. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. #Watts  
    net.avionics        = avionics      
    
    # add the solar network to the vehicle
    vehicle.append_component(net)          
 
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle



def main_wing_inputs(wing):
    
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
    
    return wing

def hstab_inputs(wing):
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
    
    return wing

def vstab_inputs(wing):
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
    return wing    


def fuselage_inputs(fuselage):
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

    return fuselage

def prop_inputs(prop):
    prop.tip_radius          = 106./2. * Units.inches 
    prop.hub_radius          = 11.1    * Units.inches 
    prop.design_Cl           = 0.8 
    prop.origin              = [[118.* Units.inches,0.0,0.0]]      
    prop.rotation            = [-1]
    
    return prop    

def bat_inputs(bat):
    bat.mass_properties.mass = 1009 * Units.kg
    
    return bat


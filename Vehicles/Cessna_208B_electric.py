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
from SUAVE.Plots.Geometry_Plots.plot_vehicle import plot_vehicle  
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

def vehicle_setup(cargo_mass=2300*Units.lb, battery_mass=2100*Units.kg):
    
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
    vehicle.mass_properties.battery_mass    = battery_mass #2000 * Units.kg
    vehicle.mass_properties.max_payload     = cargo_mass #2300. * Units.lb

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
    bat.mass_properties.mass = vehicle.mass_properties.battery_mass
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
 
    # compute the takeoff weight of the vehicle
    vehicle = calculate_takeoff_weight(vehicle)
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    ## Plot the vehicle:
    #plot_vehicle(vehicle, save_figure = False, plot_control_points = False)
    #plt.show()        

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
    wing.taper                   = wing.chords.tip / wing.chords.root
    
    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference # 9.702
    
    wing.twists.root             = 3.0 * Units.degrees # Change
    wing.twists.tip              = 1.5 * Units.degrees # Change
    
    wing.origin                  = [[130.6* Units.inches,0,50*Units.inches]]
    wing.aerodynamic_center      = [17.* Units.inches,0,60*Units.inches]
    
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
    wing.taper                   =  wing.chords.tip / wing.chords.root

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[370.33* Units.inches,0,40*Units.inches]]
    wing.aerodynamic_center      = [15.* Units.inches,0,0] # Change
    
    return wing

def vstab_inputs(wing):
    
    
    wing.sweeps.quarter_chord    = 25. * Units.deg # Change
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 3.57 * Units.meter**2 	
    wing.spans.projected         = 2.05 *Units.meter 

    wing.chords.root             = 144.695833333 * Units.inches # 2.049 * Units.meter
    wing.chords.tip              = 1.434 * Units.meter
    wing.chords.mean_aerodynamic = 1.741 * Units.meter
    wing.taper                   = wing.chords.tip / wing.chords.root

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[306.3* Units.inches,0,  0.832+12*Units.inches]]
    wing.aerodynamic_center      = [20.* Units.inches,0,0]  # Change
    
    # Segments
    segment                           = SUAVE.Components.Wings.Segment() 
    segment.tag                       = 'root'  
    segment.percent_span_location     = 0.0    
    segment.twist                     = 0. * Units.deg
    segment.root_chord_percent        = 1.
    segment.dihedral_outboard         = 0 * Units.degrees
    segment.sweeps.quarter_chord      = 72. * Units.degrees  
    segment.thickness_to_chord        = .1
    wing.append_segment(segment)      
    
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.24
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.435
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 25. * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.1895
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .1  
    wing.append_segment(segment)
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
    
    # Segment  
    segment                                     = SUAVE.Components.Fuselages.Segment() 
    segment.tag                                 = 'segment_0'  
    segment.percent_x_location                  = 0 
    segment.percent_z_location                  = 0 
    segment.height                              = fuselage.heights.at_quarter_length * 0.2
    segment.width                               = fuselage.width * 0.1
    fuselage.Segments.append(segment)             
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.04166667
    segment.percent_z_location                  = 0.00
    segment.height                              = (0.16198347/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width * 0.35
    fuselage.Segments.append(segment)    

    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.19208333
    segment.percent_z_location                  = -0.81735537e-02
    segment.height                              = (0.35231405/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width * 0.5
    fuselage.Segments.append(segment)         

    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.29166667
    segment.percent_z_location                  = 2.83471074e-02#6.07438017e-02
    segment.height                              = (0.52644628/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width
    fuselage.Segments.append(segment)      
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.46666667
    segment.percent_z_location                  = 2.83471074e-02
    segment.height                              = (0.52644628/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.60625
    segment.percent_z_location                  = 2.83471074e-02
    segment.height                              = (0.47785124/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width
    fuselage.Segments.append(segment)    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.76041667
    segment.percent_z_location                  = 4.28925620e-02
    segment.height                              = (0.34016529/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width * .5
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.87916667
    segment.percent_z_location                  = 6.50413223e-02
    segment.height                              = (0.23487603/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width * 0.25
    fuselage.Segments.append(segment)        
       
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.9625
    segment.percent_z_location                  = 0.75537190e-01
    segment.height                              = (0.1038843/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width * 0.08  
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.085 #1.57933884e-01
    segment.height                              = (0.05/0.52644628) * fuselage.heights.maximum
    segment.width                               = fuselage.width * 0.05      
    fuselage.Segments.append(segment)        

    return fuselage

def prop_inputs(prop):
    prop.tip_radius          = 106./2. * Units.inches 
    prop.hub_radius          = 11.1    * Units.inches 
    prop.design_Cl           = 0.8 
    prop.origin              = [[0.,0.0,0.0]]      
    prop.rotation            = [-1]
    
    return prop    

def bat_inputs(bat):
    bat.mass_properties.mass = 1009 * Units.kg
    
    return bat

def calculate_takeoff_weight(vehicle):
    # ------------------------------------------------------------------
    #  Weights Analysis
    # ------------------------------------------------------------------ 
    converged = False
    prior_empty = 0
    while converged == False:
        
        analyses = SUAVE.Analyses.Vehicle()
        
        weights = SUAVE.Analyses.Weights.Weights_Transport()
        weights.vehicle = vehicle
        analyses.append(weights)
            
        analyses.weights.evaluate()
        
        empty   = vehicle.weight_breakdown.empty 
        payload = vehicle.mass_properties.max_payload
        batmass = vehicle.mass_properties.battery_mass
        
        MTOW    = empty + payload + batmass
        vehicle.mass_properties.max_takeoff = MTOW
        vehicle.mass_properties.takeoff = MTOW
        vehicle.mass_properties.operating_empty = empty +batmass
        if abs(empty-prior_empty) <1e-5:
            converged = True
        else:
            prior_empty = empty
    
    return vehicle
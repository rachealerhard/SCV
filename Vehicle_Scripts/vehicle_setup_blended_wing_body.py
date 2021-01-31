# vehicle_setup_blended_wing_body.py
#
# Created: Jan 2021, N. Goodson
#
# Based on Tutorial by: E. Botero, T. MacDonald

""" Setup file for Blended wing body aircraft with same wing loading and TO mass as a Cessna208B Super Cargomaster.
    Aspect ratio and airfoils are based on SUAVE BWB tutorial (itself based on Boeing BWB-450)
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import numpy as np
import pylab as plt

import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars import compute_airfoil_polars

from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv


# TODO: Remove these imports
from SUAVE.Plots.Mission_Plots import *


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup(takeoff_mass, battery_mass, cargo_mass, e_bat):
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'AviE_BWB'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.max_takeoff               = takeoff_mass # 3985. * Units.kilogram
    vehicle.mass_properties.takeoff                   = takeoff_mass # 3985. * Units.kilogram
    vehicle.mass_properties.operating_empty           = 2533. * Units.kilogram
    vehicle.mass_properties.max_zero_fuel             = 2533. * Units.kilogram
    vehicle.mass_properties.cargo                     = cargo_mass # 1451. * Units.kilogram

    # envelope properties
    vehicle.envelope.ultimate_load = 5.7  # Safety factor of 1.5 on limit load
    vehicle.envelope.limit_load    = 3.8

    # basic parameters
    vehicle.reference_area         = 279. * Units.feet**2
    vehicle.passengers             = 9
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "short range"


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True

    wing.origin                  = [[0.,0.,0]]
    wing.dynamic_pressure_ratio  = 1.0

    # Main Wing Segments
    #----------------------------------------------------------------------
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.0
    wing.Segments.append(segment)    
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 0.052
    wing.Segments.append(segment)   

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_3'
    segment.percent_span_location = 0.138
    wing.Segments.append(segment)   
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_4'
    segment.percent_span_location = 0.221
    wing.Segments.append(segment)       
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_5'
    segment.percent_span_location = 0.457
    wing.Segments.append(segment)       
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_6'
    segment.percent_span_location = 0.568
    wing.Segments.append(segment)     
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_7'
    segment.percent_span_location = 0.97
    wing.Segments.append(segment)      

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    wing.Segments.append(segment)  

    wing = main_wing_inputs(wing)  # inputs to main wing to be optimized

    # add to vehicle
    vehicle.append_component(wing)

    # -----------------------------------------------------------------
    #   Define Propellor
    # -----------------------------------------------------------------

    # build network
    net = Battery_Propeller()
    net.number_of_engines = 1.
    net.nacelle_diameter  = 42 * Units.inches
    net.engine_length     = 0.01 * Units.inches

    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 the Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_blades = 3.0
    prop.freestream_velocity = 185. * Units.knots
    prop.angular_velocity = 1700. * Units.rpm  # Change
    prop.design_altitude = 12000. * Units.feet
    prop.design_thrust = None  # 0.0
    prop.design_power = .64 * 503 * Units.kilowatts

    prop = prop_inputs(prop)

    prop.airfoil_geometry = ["Polars\Clark_y.txt"]
    prop.airfoil_polars = [[
            "Polars\Clark_y_polar_Re_100000.txt",
            "Polars\Clark_y_polar_Re_200000.txt",
            "Polars\Clark_y_polar_Re_500000.txt",
            "Polars\Clark_y_polar_Re_1000000.txt",
        ]]

    polar_stations = np.zeros(20)
    prop.airfoil_polar_stations = list(polar_stations.astype(int))

    prop.symmetry = True
    prop = propeller_design(prop)  # optimizes propellor shape

    airfoil_polars = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)
    airfoil_cl_surs = airfoil_polars.lift_coefficient_surrogates
    airfoil_cd_surs = airfoil_polars.drag_coefficient_surrogates
    prop.airfoil_cl_surrogates = airfoil_cl_surs
    prop.airfoil_cd_surrogates = airfoil_cd_surs

    net.propeller = prop

    # Component: Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat = bat_inputs(bat,battery_mass)
    bat.specific_energy      = e_bat * 0.8 # weighted by packing factor
    bat.resistance           = 0.006
    bat.max_voltage          = 500.

    initialize_from_mass(bat, bat.mass_properties.mass)
    net.battery = bat
    net.voltage = bat.max_voltage

    # ------------------------------------------------------------------
    # Design Motors
    # ------------------------------------------------------------------
    # Propeller  motor
    # Component 4 the Motor
    motor = SUAVE.Components.Energy.Converters.Motor()
    etam = 0.95
    v = bat.max_voltage * 3 / 4
    omeg = prop.angular_velocity
    io = 4.0
    start_kv = 1
    end_kv = 25
    # do optimization to find kv or just do a linspace then remove all negative values, take smallest one use 0.05 change
    # essentially you are sizing the motor for a particular rpm which is sized for a design tip mach
    # this reduces the bookkeeping errors
    possible_kv_vals = np.linspace(start_kv, end_kv, (end_kv - start_kv) * 20 + 1, endpoint=True) * Units.rpm
    res_kv_vals = ((v - omeg / possible_kv_vals) * (1. - etam * v * possible_kv_vals / omeg)) / io
    positive_res_vals = np.extract(res_kv_vals > 0, res_kv_vals)
    kv_idx = np.where(res_kv_vals == min(positive_res_vals))[0][0]
    kv = possible_kv_vals[kv_idx]
    res = min(positive_res_vals)

    motor.mass_properties.mass = 10. * Units.kg
    motor.origin = prop.origin
    motor.propeller_radius = prop.tip_radius
    motor.speed_constant = 0.35
    motor.resistance = res
    motor.no_load_current = io
    motor.gear_ratio = 1.
    motor.gearbox_efficiency = 1.  # Gear box efficiency
    net.motor = motor

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw = 10.  # Watts
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20.  # Watts
    net.avionics = avionics

    # add the solar network to the vehicle
    vehicle.append_component(net)

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle


def main_wing_inputs(wing):
    wing.sweeps.quarter_chord    = 33. * Units.deg
    wing.thickness_to_chord      = 0.15
    wing.areas.reference         = 279. * Units.feet**2
    wing.spans.projected         = 38.6 * Units.feet  # Aspect ratio matches BWB tutorial (5.32)

    wing.chords.root             = 19.3 * Units.feet  # c/b matches tutorial
    wing.chords.tip              = 0.47  * Units.feet
    wing.chords.mean_aerodynamic = 10. * Units.feet
    wing.taper                   = 0.0138

    wing.aspect_ratio            = wing.spans.projected ** 2 / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.dihedral                = 2.5 * Units.degrees

    wing.aerodynamic_center      = [0,0,0]

    #--------------------------------------------------------------
    # Segments
    #--------------------------------------------------------------
    for segment in wing.Segments:
        if segment.tag == "section_1":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 1.
            segment.dihedral_outboard = 0. * Units.degrees
            segment.sweeps.quarter_chord = 40.0 * Units.degrees
            segment.thickness_to_chord = 0.165

        elif segment.tag == "section_2":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.921
            segment.dihedral_outboard = 0. * Units.degrees
            segment.sweeps.quarter_chord = 52.5 * Units.degrees
            segment.thickness_to_chord = 0.167

        elif segment.tag == "section_3":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.76
            segment.dihedral_outboard = 1.85 * Units.degrees
            segment.sweeps.quarter_chord = 36.9 * Units.degrees
            segment.thickness_to_chord = 0.171

        elif segment.tag == "section_4":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.624
            segment.dihedral_outboard = 1.85 * Units.degrees
            segment.sweeps.quarter_chord = 30.4 * Units.degrees
            segment.thickness_to_chord = 0.175

        elif segment.tag == "section_5":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.313
            segment.dihedral_outboard = 1.85 * Units.degrees
            segment.sweeps.quarter_chord = 30.85 * Units.degrees
            segment.thickness_to_chord = 0.118

        elif segment.tag == "section_6":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.197
            segment.dihedral_outboard = 1.85 * Units.degrees
            segment.sweeps.quarter_chord = 34.3 * Units.degrees
            segment.thickness_to_chord = 0.10

        elif segment.tag == "section_7":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.086
            segment.dihedral_outboard = 73. * Units.degrees
            segment.sweeps.quarter_chord = 55. * Units.degrees
            segment.thickness_to_chord = 0.10

        elif segment.tag == "tip":
            segment.twist = 0. * Units.deg
            segment.root_chord_percent = 0.0241
            segment.dihedral_outboard = 0. * Units.degrees
            segment.sweeps.quarter_chord = 0. * Units.degrees
            segment.thickness_to_chord = 0.10

    return wing


def prop_inputs(prop):
    prop.tip_radius = 106. / 2. * Units.inches
    prop.hub_radius = 11.1 * Units.inches
    prop.design_Cl = 0.8
    prop.origin = [[118. * Units.inches, 0.0, 0.0]]
    prop.rotation = [-1]

    return prop


def bat_inputs(bat, battery_mass):
    bat.mass_properties.mass = battery_mass  # 1009 * Units.kg  # optimize how much battery weight we bring to maximize range

    return bat

# blended_wing_body.py
#
# Created: Jan 2021, N. Goodson
#
# Based on Tutorial by: E. Botero, T. MacDonald

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import numpy as np
import pylab as plt

import SUAVE
from SUAVE.Core import Data, Units

from SUAVE.Input_Output.OpenVSP import write
from SUAVE.Input_Output.OpenVSP.get_vsp_areas import get_vsp_areas

from SUAVE.Plots.Mission_Plots import *

from vehicle_setup_blended_wing_body import vehicle_setup # Vehicle_Scripts.



# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights

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
    takeoff_mass = 3985. * Units.kilogram
    battery_mass = 1009. * Units.kilogram
    cargo_mass = 1451. * Units.kilogram
    e_bat = 450. * Units.Wh / Units.kilogram

    # vehicle data
    vehicle = vehicle_setup(takeoff_mass, battery_mass, cargo_mass, e_bat)
    configs = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission = mission_setup(configs_analyses, vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag, config in configs.items():
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
    weights = SUAVE.Analyses.Weights.Weights_BWB()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis

    # aerodynamics = SUAVE.Analyses.Aerodynamics.SU2_Euler()
    # aerodynamics.geometry = vehicle
    # aerodynamics.process.compute.lift.inviscid.training_file = 'bwb_base_data.txt'

    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # OpenVSP mesh parameters
    wing_segments = vehicle.wings.main_wing.Segments
    wing_segments.section_1.vsp_mesh = Data()
    wing_segments.section_1.vsp_mesh.inner_radius = 4.
    wing_segments.section_1.vsp_mesh.outer_radius = 4.
    wing_segments.section_1.vsp_mesh.inner_length = .14
    wing_segments.section_1.vsp_mesh.outer_length = .14

    wing_segments.section_2.vsp_mesh = Data()
    wing_segments.section_2.vsp_mesh.inner_radius = 4.
    wing_segments.section_2.vsp_mesh.outer_radius = 4.
    wing_segments.section_2.vsp_mesh.inner_length = .14
    wing_segments.section_2.vsp_mesh.outer_length = .14

    wing_segments.section_3.vsp_mesh = Data()
    wing_segments.section_3.vsp_mesh.inner_radius = 4.
    wing_segments.section_3.vsp_mesh.outer_radius = 4.
    wing_segments.section_3.vsp_mesh.inner_length = .14
    wing_segments.section_3.vsp_mesh.outer_length = .14

    wing_segments.section_4.vsp_mesh = Data()
    wing_segments.section_4.vsp_mesh.inner_radius = 4.
    wing_segments.section_4.vsp_mesh.outer_radius = 2.8
    wing_segments.section_4.vsp_mesh.inner_length = .14
    wing_segments.section_4.vsp_mesh.outer_length = .14

    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
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

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # write(vehicle, base_config.tag)  # Write to OpenVSP

    return configs


def simple_sizing(configs):
    base = configs.base
    base.pull_base()

    # Areas
    # wetted_areas = get_vsp_areas(base.tag)

    for wing in base.wings:
        wing.areas.wetted   = 1.75 * wing.areas.reference
        wing.areas.exposed  = 0.8  * wing.areas.wetted
        wing.areas.affected = 0.6  * wing.areas.wetted
        # wing.areas.wetted = wetted_areas[wing.tag]
        # wing.areas.exposed = wetted_areas[wing.tag]
        # wing.areas.affected = 0.6 * wing.areas.wetted

        # diff the new data
    base.store_diff()

    return


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses, vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude = 0.0 * Units.ft
    airport.delta_isa = 0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points = 4
    base_segment.process.iterate.unknowns.network = vehicle.propulsors.battery_propeller.unpack_unknowns
    base_segment.process.iterate.residuals.network = vehicle.propulsors.battery_propeller.residuals
    base_segment.state.unknowns.propeller_power_coefficient = 0.005 * ones_row(1)
    base_segment.state.unknowns.battery_voltage_under_load = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(
        1)
    base_segment.state.residuals.network = 0. * ones_row(2)

    # ------------------------------------------------------------------
    #   First Climb Segment: constant Speed, constant rate segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend(analyses.base)

    segment.altitude_start = 0. * Units.meter
    segment.altitude_end = 2000. * Units.meter
    segment.air_speed = 125. * Units.mph
    segment.climb_rate = 1000. * Units['ft/min']  # max climb rate for the Cessna Caravan is 1234 ft/min
    segment.battery_energy = vehicle.propulsors.battery_propeller.battery.max_energy * 0.89
    segment.state.unknowns.throttle = 0.85 * ones_row(1)

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant rate segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend(analyses.base)

    segment.altitude_start = 2000. * Units.meter
    segment.altitude_end = 4500. * Units.meter
    segment.air_speed = 150. * Units.mph
    segment.climb_rate = 1000. * Units['ft/min']  # max climb rate for the Cessna Caravan
    segment.state.unknowns.throttle = 0.85 * ones_row(1)

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend(analyses.base)

    segment.altitude = 4500. * Units.meter  # 7620.  * Units.meter
    segment.air_speed = 180. * Units.mph
    segment.distance = 75. * Units.kilometer
    segment.state.unknowns.throttle = 0.9 * ones_row(1)

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Descent Segment: constant Speed, constant rate segment
    # ------------------------------------------------------------------
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "decent"
    segment.analyses.extend(analyses.base)
    segment.altitude_start = 4500. * Units.meter  # 7620.  * Units.meter
    segment.altitude_end = 2500 * Units.meter
    segment.air_speed = 140. * Units['mph']
    segment.climb_rate = - 500. * Units['ft/min']
    segment.state.unknowns.throttle = 0.9 * ones_row(1)

    # add to misison
    mission.append_segment(segment)

    return mission


def missions_setup(base_mission):
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions


# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results, line_style='bo-'):
    axis_font = {'fontname': 'Arial', 'size': '14'}

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


# def plot_mission(results, line_style='bo-'):
#     # Plot Aerodynamic Forces
#     plot_aerodynamic_forces(results, line_style)
#
#     # Plot Aerodynamic Coefficients
#     plot_aerodynamic_coefficients(results, line_style)
#
#     # Drag Components
#     plot_drag_components(results, line_style)
#
#     # Plot Altitude, sfc, vehicle weight
#     plot_altitude_sfc_weight(results, line_style)
#
#     # Plot Velocities
#     plot_aircraft_velocities(results, line_style)
#
#     return


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


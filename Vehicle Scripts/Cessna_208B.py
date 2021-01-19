# Cessna_208B.py
# 
# Created:  Jan 2021, R. Erhard

""" setup file for Cessna 208B aircraft
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Plots.Mission_Plots import *

from vehicle_setup_Cessna208B import vehicle_setup


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
    
    # ------------------------------------------------------------------
    #   First Climb Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 2000. * Units.meter
    segment.air_speed      = 125.  * Units.mph
    segment.climb_rate     = 1234.  * Units['ft/min'] # max climb rate for the Cessna Caravan
    
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
    segment.climb_rate     = 1234. * Units['ft/min'] # max climb rate for the Cessna Caravan
    
    # add to misison
    mission.append_segment(segment)
     

    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Speed, constant rate segment 
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 4500. * Units.meter
    segment.altitude_end   = 7000. * Units.meter
    segment.air_speed      = 180.  * Units.mph
    segment.climb_rate     = 1234. * Units['ft/min'] # max climb rate for the Cessna Caravan
    
    # add to misison
    mission.append_segment(segment)
      
    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 7620.  * Units.meter
    segment.air_speed = 214.   * Units.mph
    segment.distance  = 1900.   * Units.kilometer

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

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results,line_style='bo-'):

    axis_font = {'fontname':'Arial', 'size':'14'}  
    line_color = 'bo-'    
    #------------------------------------------------
    # Plot Flight Conditions 
    #------------------------------------------------
    axis_font = {'size':'14'} 
    fig = plt.figure()
    fig.set_size_inches(12, 10)
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        airspeed = segment.conditions.freestream.velocity[:,0] 
        theta    = segment.conditions.frames.body.inertial_rotations[:,1,None] / Units.deg
        
        x        = segment.conditions.frames.inertial.position_vector[:,0]
        y        = segment.conditions.frames.inertial.position_vector[:,1]
        z        = segment.conditions.frames.inertial.position_vector[:,2]
        altitude = segment.conditions.freestream.altitude[:,0]
        
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, altitude, line_color)
        axes.set_ylabel('Altitude (m)',axis_font)
        set_axes(axes)            

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , airspeed , line_color )
        axes.set_ylabel('Airspeed (m/s)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(2,2,3)
        axes.plot( time , theta, line_color )
        axes.set_ylabel('Pitch Angle (deg)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)   
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , x/1000, 'bo-') #, time , y, 'go-' , time , z, 'ro-')
        axes.set_ylabel('Range (km)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)         
        
    
    
    #------------------------------------------------
    # Plot Aerodynamic Forces 
    #------------------------------------------------
    axis_font = {'size':'14'}
    fig = plt.figure()
    fig.set_size_inches(12, 10)
    
    for segment in results.segments.values():
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]  
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]          
        eta    = segment.conditions.propulsion.throttle[:,0]

        axes = fig.add_subplot(2,2,1)
        axes.plot( time , eta , line_color )
        axes.set_ylabel('Throttle',axis_font)
        set_axes(axes)	 

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , Lift , line_color)
        axes.set_ylabel('Lift (N)',axis_font)
        set_axes(axes)
        
        axes = fig.add_subplot(2,2,3)
        axes.plot( time , Thrust , line_color)
        axes.set_ylabel('Thrust (N)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , Drag , line_color)
        axes.set_ylabel('Drag (N)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)      
    

    #------------------------------------------------
    # Plot Aerodynamic Coefficients 
    #------------------------------------------------
    axis_font = {'size':'14'}  
    fig = plt.figure()
    fig.set_size_inches(12, 10)
    
    for segment in results.segments.values(): 
        time = segment.conditions.frames.inertial.time[:,0] / Units.min
        cl   = segment.conditions.aerodynamics.lift_coefficient[:,0,None] 
        cd   = segment.conditions.aerodynamics.drag_coefficient[:,0,None] 
        aoa  = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d  = cl/cd

        axes = fig.add_subplot(2,2,1)
        axes.plot( time , aoa , line_color )
        axes.set_ylabel('Angle of Attack (deg)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , cl, line_color )
        axes.set_ylabel('CL',axis_font)
        set_axes(axes)   
        
        axes = fig.add_subplot(2,2,3)
        axes.plot( time , cd, line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('CD',axis_font)
        set_axes(axes)   
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , l_d, line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('L/D',axis_font)
        set_axes(axes)            

    
    #------------------------------------------------    
    # Drag Components
    #------------------------------------------------
    axis_font = {'size':'14'} 
    fig    = plt.figure()
    fig.set_size_inches(12, 10)     
    axes = fig.add_subplot(1,1,1) 
    
    for i, segment in enumerate(results.segments.values()):
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]
        
        axes.plot( time , cdp , 'ko-', label='CD parasite' )
        axes.plot( time , cdi , line_color, label='CD induced' )
        axes.plot( time , cdc , 'go-', label='CD compressibility' )
        axes.plot( time , cdm , 'yo-', label='CD miscellaneous' )
        axes.plot( time , cd  , 'ro-', label='CD total'   )            
        if i == 0: 
            axes.legend(loc='upper center')   
            
    axes.set_xlabel('Time (min)',axis_font)
    axes.set_ylabel('CD',axis_font)
    axes.grid(True)  

    #------------------------------------------------    
    # Plot Altitude, sfc, vehicle weight 
    #------------------------------------------------
    axis_font = {'size':'14'} 
    fig = plt.figure()
    fig.set_size_inches(10, 8) 
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min 
        mass     = segment.conditions.weights.total_mass[:,0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft
        mdot     = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust   =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc      = (mdot / Units.lb) / (thrust /Units.lbf) * Units.hr

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , altitude , line_color)
        axes.set_ylabel('Altitude (ft)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , sfc , line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('sfc (lb/lbf-hr)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , mass , 'ro-' )
        axes.set_ylabel('Weight (lb)',axis_font)
        set_axes(axes)
        
    
    #------------------------------------------------
    # Plot Velocities 
    #------------------------------------------------  
    axis_font = {'size':'14'}  
    fig = plt.figure()
    fig.set_size_inches(10, 8) 
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min 
        velocity = segment.conditions.freestream.velocity[:,0] 
        density  = segment.conditions.freestream.density[:,0]
        EAS      = velocity * np.sqrt(density/1.225)
        mach     = segment.conditions.freestream.mach_number[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , velocity / Units.kts, line_color)
        axes.set_ylabel('velocity (kts)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , EAS / Units.kts, line_color)
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Equivalent Airspeed',axis_font)
        set_axes(axes)    
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach , line_color)
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Mach',axis_font)
        set_axes(axes)  

        
    return
def plots(results,line_style='bo-'):

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
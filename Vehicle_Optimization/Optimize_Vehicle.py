# Optimize.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

from SUAVE.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
import SUAVE.Optimization.Package_Setups.pyopt_setup as pyopt_setup
from SUAVE.Optimization.Nexus import Nexus
import time
# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    '''
   
    '''
    
    problem = setup()
    
    start_t = time.time()
    output  = scipy_setup.SciPy_Solve(problem)
    end_t = (time.time() - start_t)/60 
    print(f"\n\n\nElapsed time: {end_t :.2f} [min]")
    print(f"Mission range: {problem.summary.mission_range/1000 :.2f} [km]")
    print(f"Total aircraft weight: {problem.summary.total_weight :.2f} [kg]")
    print(f"Battery mass required: {problem.optimization_problem.inputs[0][1] :.2f} [kg]")
    print(f"Payload: {problem.summary.payload :.2f} [kg]")
    print(f"Wing area: {problem.optimization_problem.inputs[1][1] :.2f} [m^2]")
    print(f"Taper ratio: {problem.optimization_problem.inputs[2][1] :.2f}")
    problem.translate(output)

    Plot_Mission.plot_mission(problem.results.mission)
    
    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    # [ tag , initial, [lb,ub], scaling, units ]
    problem.inputs = np.array([   
        [ 'wing_area'          ,    30.    , (   22. ,   45.), 100.0 , Units.meter**2   ],   
        [ 'wing_aspect_ratio'  ,     9.702 , (    5. ,   12.), 10.0 , Units.less       ],   
        [ 'wing_sweep'         ,     0.    , (  -10.*Units.deg ,   35.*Units.deg), 1.0 , Units.rad  ], 
        [ 'wing_taper_ratio'  ,     0.7 ,    (  0.5 ,   1.), 1.0 , Units.less       ],
        ]) 

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'energy_usage', 1e8, Units.J], 
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------

    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'battery_remaining', '>', 0.15, 1., Units.less],
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]
    
    problem.aliases = [
        #[ 'bat_mass'          , ['vehicle_configurations.*.mass_properties.battery_mass',
                           #'vehicle_configurations.*.propulsors.battery_propeller.battery.mass_properties.mass'] ],
        [ 'wing_area'         , ['vehicle_configurations.*.wings.main_wing.areas.reference',
                                 'vehicle_configurations.*.reference_area']],
        [ 'wing_aspect_ratio' , 'vehicle_configurations.*.wings.main_wing.aspect_ratio'  ],
        [ 'wing_sweep'        , 'vehicle_configurations.*.wings.main_wing.sweeps.quarter_chord'  ], 
        [ 'wing_taper_ratio'        , 'vehicle_configurations.*.wings.main_wing.taper'  ], 
        #[ 'cruise_alt'      , 'missions.mission.segments.cruise.altitude'  ],
        #[ 'total_weight'      , 'summary.total_weight'  ],
        [ 'energy_usage'      , 'summary.energy_usage'                    ],
        [ 'battery_remaining' , 'summary.battery_remaining'                    ],
        #[ 'Nothing'           , 'summary.nothing'       ],
        #[ 'mission_time'      , 'summary.mission_range'                         ],
        #[ 'objective'         , 'summary.objective' ], 
    ]         
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
       
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses,nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    return nexus


if __name__ == '__main__':
    main()
    
    
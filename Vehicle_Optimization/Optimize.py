# Optimize.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

from SUAVE.Core import Units, Data
import numpy as np
import argparse
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
def main(args):
    '''
    
    '''
    problem = setup(args.Vehicle)
    
    start_t = time.time()
    output  = scipy_setup.SciPy_Solve(problem)
    end_t = (time.time() - start_t)/60 
    print(f"\n\n\nElapsed time: {end_t :.2f} [min]")
    print(f"\nMission range: {problem.summary.mission_range/1000 :.2f} [km]")
    print(f"\nTotal aircraft weight: {problem.summary.total_weight :.2f} [kg]")
    print(f"\nBattery mass required: {problem.optimization_problem.inputs[0][1] :.2f} [kg]")
    print(f"\nPayload: {problem.summary.payload :.2f} [kg]")
    problem.translate(output)

    Plot_Mission.plot_mission(problem.results.mission)
    
    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup(vehicle_name):

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    # [ tag , initial, [lb,ub], scaling, units ]
    problem.inputs = np.array([
        [ 'bat_mass'       ,   1500.  , ( 800. , 1600.), 1000.0 , Units.kg      ],        
        ]) 

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'Nothing', 1.0, Units.kg],
        #[ 'total_weight', 1.0, Units.kg], 
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------

    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'battery_remaining', '>', 0.30, 1., Units.less],
        [ 'battery_remaining', '<', 0.35, 1., Units.less],
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]
    problem.aliases = [
        #[ 'wing_area'         , 'vehicle_configurations.*.wings.main_wing.areas.reference'  ],
        #[ 'wing_aspect_ratio' , 'vehicle_configurations.*.wings.main_wing.aspect_ratio'  ],
        #[ 'wing_sweeps'         , 'vehicle_configurations.*.wings.main_wing.sweeps.quarter_chord'  ], 
        #[ 'cruise_alt'      , 'missions.mission.segments.cruise.altitude'  ],
        [ 'bat_mass'    , 'vehicle_configurations.*.mass_properties.battery_mass'  ],
        [ 'total_weight'    , 'summary.total_weight'  ],
        #[ 'energy_usage' , 'summary.energy_usage'                    ],
        [ 'battery_remaining' , 'summary.battery_remaining'                    ],
        [ 'Nothing'          ,  'summary.nothing'       ]
        #[ 'mission_time'      , 'summary.mission_range'                         ],
    ]      
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup(vehicle_name)
    
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
    parser = argparse.ArgumentParser(description='Run the SUAVE optimization')
    parser.add_argument('Vehicle',
                        metavar='vehicle',
                        type=str,
                        help='the name of the vehicle file',
                        nargs='?',
                        default='Cessna_208B_electric')
    args = parser.parse_args()
    main(args)
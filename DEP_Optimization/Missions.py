# Missions.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units
import time

import numpy as np

# Import missions:
import sys
sys.path.append('../Missions')
#from cruise_mission_variable_cruise_range import cruise_mission_setup
#from full_mission_variable_cruise_range   import full_mission_setup
from cruise_mission_fixed_mission_profile import cruise_mission_setup
from full_mission_fixed_mission_profile   import full_mission_setup



# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def setup(analyses,vehicle):
    
    #mission = full_mission_setup(vehicle, analyses) # vehiclde configs

    mission = cruise_mission_setup(vehicle, analyses) # vehiclde configs    
    return mission

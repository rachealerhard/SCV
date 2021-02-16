# Vehicles.py
# 
# Created:  Jan 2021, R. Erhard
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    
import SUAVE
from SUAVE.Core import Units, Data

import importlib
import sys
sys.path.append('../Vehicles')
# from Cessna_208B_electric import vehicle_setup

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------
def import_vehicle(vehicle_name):
    """
    Imports the setup function for the correct vehicle using
    the vehicles name
    """
    vehicle_module = importlib.import_module(vehicle_name)
    print(f"Vehicle setup function loaded for {vehicle_name}")
    return vehicle_module.vehicle_setup


def setup(vehicle_name):

    vehicle_setup_function = import_vehicle(vehicle_name)
    base_vehicle = vehicle_setup_function()
    configs = configs_setup(base_vehicle)
    
    return configs
    

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
# c208b_weights.py
# 
# Created:  Jan 2021, R. Erhard

""" simple weights estimation example for Cessna 208b aircraft geometry
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Plots.Mission_Plots import *


# Use converted Cessna 208B Electric Aircraft:
from vehicle_setup_Cessna_208B_electric import vehicle_setup_electric

from SUAVE.Methods.Weights.Correlations.FLOPS.wing import wing_weight_FLOPS
from SUAVE.Methods.Weights.Correlations.FLOPS.fuselage import fuselage_weight_FLOPS

import numpy as np
import pylab as plt


def main():
    
    vehicle = vehicle_setup_electric(0.0)
    
    # Weights settings:
    settings = Data()
    settings.FLOPS = Data()
    settings.FLOPS.aeroelastic_tailoring_factor = 0.0 # no aeroelastic tailoring
    settings.FLOPS.strut_braced_wing_factor     = 0.0 # no strut bracing factor
    settings.FLOPS.composite_utilization_factor = 0.0 # no composites
    
    vehicle.systems.accessories = "short_range"
    
    #---------------------------------------------------    
    # Weights estimate of fuselage:
    #---------------------------------------------------
    
    fuselage_weight = fuselage_weight_FLOPS(vehicle)
    
    #---------------------------------------------------
    # Weights estimate of main wing:
    #---------------------------------------------------

    WPOD = 10.
    complexity = 'Simple'
    num_main_wings = 1
    vehicle.wings.main_wing.flap_ratio  = 0.3
    
    main_wing_weight = wing_weight_FLOPS(vehicle, vehicle.wings.main_wing, WPOD, complexity, settings, num_main_wings)
        

    #---------------------------------------------------        
    # Weights estimate of horizontal tail:
    #---------------------------------------------------
    WPOD = 10.
    complexity = 'Simple'
    num_main_wings = 1
    vehicle.wings.horizontal_stabilizer.flap_ratio  = 0.3
    h_tail_weight = wing_weight_FLOPS(vehicle, vehicle.wings.horizontal_stabilizer, WPOD, complexity, settings, num_main_wings)
        

    #---------------------------------------------------        
    # Weights estimate of vertical tail:
    #---------------------------------------------------        
    WPOD = 10.
    complexity = 'Simple'
    num_main_wings = 1
    vehicle.wings.vertical_stabilizer.flap_ratio  = 0.3
    v_tail_weight = wing_weight_FLOPS(vehicle, vehicle.wings.vertical_stabilizer, WPOD, complexity, settings, num_main_wings)
        
        
    
    structural_weight = fuselage_weight +v_tail_weight +h_tail_weight + main_wing_weight
    
    #---------------------------------------------------    
    # Weights estimate of vertical tail:
    #---------------------------------------------------    
    
    return

if __name__ == '__main__': 
    main()    

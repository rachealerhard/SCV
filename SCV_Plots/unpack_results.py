# Reading pickled results:

import SUAVE
from SUAVE.Core import Units
import numpy as np
import pickle
import pylab as plt


def main():
    #data_location = ''
    #get_payload_range_diagram()
    
    get_mtow_range_diagram()
    
    return


def get_payload_range_diagram():
    
    # Results from running payload_range_tradeoff.py
    
    filename = 'payload_range_results_250Wh_kg.pkl'
    with open('C:/Users/rerha/Desktop/SCV_Class/SCV/SCV_Plots/' +filename, "rb") as file:
        res = pickle.load(file)
    
    fig = plt.figure()
    axes = fig.add_subplot(1, 1, 1)
    axes.plot(res.range_w_reserve/Units.kilometer, res.battery_mass, "o-", color='tab:blue', label="Battery Mass")
    axes.plot(res.range_w_reserve/Units.kilometer, res.cargo_mass, "o-", color='tab:red', label="Cargo Mass") 
    axes.set_xlabel("Range [km]")
    axes.set_ylabel("Mass [kg]")
    axes.set_title("Payload vs Range Diagram")
    plt.legend()  
    plt.grid()
    plt.show()
    
    return

def get_mtow_range_diagram():
    
    # Results from running payload_range_tradeoff.py
    
    filename = 'mtow_range_results_250Wh_kg.pkl'
    with open('C:/Users/rerha/Desktop/SCV_Class/SCV/SCV_Plots/' +filename, "rb") as file:
        res_250Wh_kg = pickle.load(file)
        
    filename = 'mtow_range_results_350Wh_kg.pkl'
    with open('C:/Users/rerha/Desktop/SCV_Class/SCV/SCV_Plots/' +filename, "rb") as file:
        res_350Wh_kg = pickle.load(file)        
            
    filename = 'mtow_range_results_450Wh_kg.pkl'
    with open('C:/Users/rerha/Desktop/SCV_Class/SCV/SCV_Plots/' +filename, "rb") as file:
        res_450Wh_kg = pickle.load(file)   
    
    
    # grayscale
    
            
    fig = plt.figure()
    axes = fig.add_subplot(1, 1, 1)
    axes.plot(res_250Wh_kg.range_w_reserve/Units.kilometer, res_250Wh_kg.mtows, "o-", color='tab:red', label="250 Wh/kg") 
    axes.plot(res_350Wh_kg.range_w_reserve/Units.kilometer, res_350Wh_kg.mtows, "o-", color='tab:blue', label="350 Wh/kg") 
    axes.plot(res_450Wh_kg.range_w_reserve[0:7]/Units.kilometer, res_450Wh_kg.mtows[0:7], "o-", color='tab:green', label="450 Wh/kg") 
    #axes.plot(172,3985,'*',markersize=10,color='black',label='Direct Conversion')
    #axes.plot(272,3985,'*',markersize=10,color='black')
    #axes.plot(372,3985,'*',markersize=10,color='black')
    #axes.plot(np.array([125, 375]), np.array([3985, 3985]), "k",label="Cessna Caravan Baseline")
    axes.set_xlabel("Range [km]")
    axes.set_ylabel("Takeoff Weight [kg]")
    axes.set_title("Takeoff Weight vs. Range \n(Fixed Cargo and Structural Weight)")
    axes.set_ylim([0,5500])
    plt.legend()
    plt.grid()
    plt.show()
    
    
    fig = plt.figure()
    axes = fig.add_subplot(1, 1, 1)
    axes.plot(res_250Wh_kg.range_w_reserve/Units.kilometer, res_250Wh_kg.battery_masses*250./1000, "o-", color='tab:red', label="250 Wh/kg") 
    axes.plot(res_350Wh_kg.range_w_reserve/Units.kilometer, res_350Wh_kg.battery_masses*350./1000, "o-", color='tab:blue', label="350 Wh/kg") 
    axes.plot(res_450Wh_kg.range_w_reserve[0:7]/Units.kilometer, res_450Wh_kg.battery_masses[0:7]*450./1000, "o-", color='tab:green', label="450 Wh/kg") 
    axes.set_xlabel("Range [km]")
    axes.set_ylabel("Total Energy [kWh]")
    axes.set_title("Total Energy vs. Range \n(Fixed Cargo and Structural Weight)")
    axes.set_ylim([0,600])
    plt.legend()
    plt.grid()
    plt.show()   
    
    # Economics:
    cost_per_kWh = 13.19 # cents
    
    
    return

if __name__ == "__main__":
    main()


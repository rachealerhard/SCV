# Reading pickled results:

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import pickle
import pylab as plt


def main():
    # save results to file:
    res250 = Data()
    res250.battery_mass = np.array([850, 1050, 1250, 1450, 1650, 1850, 2050, 2250])
    res250.mtows = np.array([4426.262451, 4626.262451, 4826.262451, 5026.262451, 5226.262, 5426.262, 5626.262, 5826.262])
    res250.range_w_reserve = np.array([124857.7518, 158243.2181, 188051.5202, 214913.3186, 238902.4, 260263.26, 279244.5063, 296043.3926])
    
    res350 = Data()
    res350.battery_mass = np.array([550, 600, 833, 1031, 1229, 1400, 1823])
    res350.mtows = np.array([4126.262, 4176.262, 4409.262, 4607.262, 4805.262, 4976.262, 5399.262])
    res350.range_w_reserve = np.array([124857.7518, 138788.857, 200980.5719, 247879.2901, 289510.2057, 321449.361, 391013.0222])
        
    res450 = Data()
    res450.battery_mass = np.array([400, 500, 660, 820, 980, 1140, 1200, 1245, 1350, 1550, 1650])
    res450.mtows = np.array([3976.262, 4076.262451, 4236.262451, 4396.262451, 4556.262451, 4716.262451, 4776.262451, 4821.262, 4926.262, 5126.262, 5226.262])
    res450.range_w_reserve = np.array([124857.7518, 161986.1937, 222468.22, 276911.859, 326269.78, 371119.1126, 387052.03, 397464.17, 424223.7, 468924.9, 490226.3])
    
    results_2300lb_payload = Data()
    results_2300lb_payload.e_250 = res250
    results_2300lb_payload.e_350 = res350
    results_2300lb_payload.e_450 = res450
    
    filename = 'results_2300lb_payload'
    save_results(results_2300lb_payload, filename)
    get_mtow_range_diagram()
    
    return



def save_results(results, filename):
    save_file = filename + '.pkl'
    with open('C:/Users/rerha/Desktop/SCV_Class/SCV/Results/mtow_v_range/' + save_file, 'wb') as file:
        pickle.dump(results, file)
    return


def get_mtow_range_diagram():
    
    # Results from running payload_range_tradeoff.py
    
    filename = 'results_2300lb_payload.pkl'
    with open('C:/Users/rerha/Desktop/SCV_Class/SCV/Results/mtow_v_range/' +filename, "rb") as file:
        res = pickle.load(file)

    
    # grayscale
    
            
    fig = plt.figure()
    axes = fig.add_subplot(1, 1, 1)
    axes.plot(res.e_250.range_w_reserve/Units.kilometer, res.e_250.mtows, "--", color='tab:red',   linewidth=3, label="250 Wh/kg") 
    axes.plot(res.e_350.range_w_reserve/Units.kilometer, res.e_350.mtows, ".-.", color='tab:blue',  linewidth=3, label="350 Wh/kg") 
    axes.plot(res.e_450.range_w_reserve/Units.kilometer, res.e_450.mtows, ":", color='tab:green', linewidth=4, label="450 Wh/kg") 
    axes.set_xlabel("Range [km]")
    axes.set_ylabel("Takeoff Weight [kg]")
    axes.set_title("Takeoff Weight vs. Range at Max Payload")
    axes.set_ylim([0,6500])
    plt.legend()
    plt.grid()
    #plt.show()
    
    
    fig = plt.figure()
    axes = fig.add_subplot(1, 1, 1)
    axes.plot(res.e_250.range_w_reserve/Units.kilometer, res.e_250.battery_mass*250./1000, "--", color='tab:red',   linewidth=3, label="250 Wh/kg") 
    axes.plot(res.e_350.range_w_reserve/Units.kilometer, res.e_350.battery_mass*350./1000, ".-.", color='tab:blue',  linewidth=3, label="350 Wh/kg") 
    axes.plot(res.e_450.range_w_reserve/Units.kilometer, res.e_450.battery_mass*450./1000, ":", color='tab:green', linewidth=4, label="450 Wh/kg") 
    axes.set_xlabel("Range [km]")
    axes.set_ylabel("Total Energy [kWh]")
    axes.set_title("Total Energy vs. Range at Max Payload")
    axes.set_ylim([0,800])
    plt.legend()
    plt.grid()
    plt.show()   
    
    # Economics:
    cost_per_kWh = 13.19 # cents
    
    
    return

if __name__ == "__main__":
    main()


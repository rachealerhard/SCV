import SUAVE

from SUAVE.Core import Units, Data
import numpy as np
import matplotlib.pyplot as plt
import pickle

# save results and plot:

def main():
    payload_range_results()
    
    return
    
def payload_range_results():
    
    results = Data()
    results.case_1 = Data()
    results.case_2 = Data()
    results.case_3 = Data()
    results.case_4 = Data()

    #results.case_0.payload =     
    results.case_1.payload = 2300 * Units.lb # fixed
    results.case_2.payload = 1300 * Units.lb # fixed
    results.case_3.payload = 1764.262451 * Units.kg
    results.case_4.payload = np.array([2300, 2000, 1000, 0]) * Units.lb # fixed

    #results.case_0.battery_mass =   
    results.case_1.battery_mass = 2311.00 * Units.kg
    results.case_2.battery_mass = 2764.59 * Units.kg
    results.case_3.battery_mass = 1325 * Units.kg
    results.case_4.battery_mass = np.array([1009., 1009., 1009., 1009.]) * Units.kg # fixed

    #results.case_0.max_range_w_reserve =   
    results.case_1.max_range_w_reserve = 530.68 * Units.km # fixed
    results.case_2.max_range_w_reserve = 682.93 * Units.km 
    results.case_3.max_range_w_reserve = 300.0 * Units.km # fixed
    results.case_4.max_range_w_reserve = np.array([242.13, 260.26, 326.99, 402.28]) * Units.km
    
    #results.case_0.energy_used =   
    results.case_1.energy_used = 568.48 * Units.kWh
    results.case_2.energy_used = 712.96 * Units.kWh
    results.case_3.energy_used = 300.0 * Units.kWh
    results.case_4.energy_used = np.array([207.425, 211.12, 222.287, 231.80]) * Units.kWh    

    #results.case_0.total_energy =  
    results.case_1.total_energy = 831.96 * Units.kWh
    results.case_2.total_energy = 995.25 * Units.kWh
    results.case_3.total_energy = 300.0 * Units.kWh
    results.case_4.total_energy = np.array([363.24, 363.24, 363.24, 363.24]) * Units.kWh
    
    
    filename = 'File Name'
    save_results(results, filename)    
    # ---------------------------------------------------------------
    # Plot the payload-range diagram for cases 1-3: 
    # Fixed takeoff weight, battery swapping
    # ---------------------------------------------------------------
    fixed_TOW_ranges = np.array([results.case_3.max_range_w_reserve,
                    results.case_1.max_range_w_reserve,
                    results.case_2.max_range_w_reserve])
    fixed_TOW_payloads = np.array([results.case_3.payload,
                    results.case_1.payload,
                    results.case_2.payload])
    R   = np.insert(fixed_TOW_ranges, 0, 0)
    PLD = np.insert(fixed_TOW_payloads, 0, results.case_3.payload)
    cases = ['Case 3','Case 1','Case 2']
    
    figure = plt.figure()
    plt.plot(R/1000, PLD, 'r', label='TOW: 5618kg')
    plt.plot(fixed_TOW_ranges/1000, fixed_TOW_payloads, 'ro')
    
    # Label points:
    i=1
    for case in cases:
        r_val = R[i]/1000
        pld_val = PLD[i]
        plt.text(r_val,pld_val, case)
        i = i +1
        
    plt.xlabel('Range (km)')
    plt.ylabel('Payload (kg)')
    plt.title('Payload Range Diagram\n(Fixed Takeoff Weight)')
    plt.grid(True)
    plt.legend()
    
        
    # ---------------------------------------------------------------
    # Plot the payload-range diagram for case 4: 
    # Fixed battery weight, variable payload
    # ---------------------------------------------------------------
    R   = np.insert(results.case_4.max_range_w_reserve, 0, 0)
    PLD = np.insert(results.case_4.payload, 0, results.case_4.payload[0])
    cases = ['Case 4.1','Case 4.2','Case 4.3']
    
    figure = plt.figure()
    plt.plot(R/1000, PLD, 'r', label='Battery Mass: 1009kg')
    plt.plot(results.case_4.max_range_w_reserve[0:-1]/1000, results.case_4.payload[0:-1], 'ro')
    
    # Label points:
    i=1
    for case in cases:
        r_val = R[i]/1000
        pld_val = PLD[i]
        plt.text(r_val,pld_val, case)
        i = i +1
        
    plt.xlabel('Range (km)')
    plt.ylabel('Payload (kg)')
    plt.title('Payload Range Diagram\n(Fixed Battery Mass)')
    plt.grid(True)
    plt.legend()
    plt.show()       
    
    
    return results

def save_results(results, filename):
    save_file = filename + '.pkl'
    with open('C:/Users/rerha/Desktop/' + save_file, 'wb') as file:
        pickle.dump(results, file)
    return

if __name__ == '__main__':
    main()

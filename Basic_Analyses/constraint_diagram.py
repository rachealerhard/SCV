# constraint_diagram.py
# 
# Created:  Jan 2021, R. Erhard

""" 
preliminary constraint diagram for Project AviE

    Source: Mattingly, J.D., Heiser, W. H., and Pratt, D. T., "Aircraft Engine Design", 2nd ed.
    AIAA Education Series, 2002. 
    
"""

import SUAVE
from SUAVE.Core import Units

import numpy as np
import pylab as plt

# Constraint Diagram:
def main():
    
    # Wing loading (weight/area)
    W_S = np.linspace(150,5000,100) # [N/m^2]
    
    # Fixed input parameters:
    AR = 9.7
    e  = 0.85
    K1 = 1 / (np.pi*AR*e) # Quadratic drag polar coefficient
    K2 = 0.00 # Linear drag polar coefficient

    
    # ----------------------------------------------------------------
    # Cruise: Constant altitude/speed, (Ps=0)
    # ---------------------------------------------------------------- 
    alt = 7620 * Units.meter
    Vcruise = 190 * Units.mph    
    T_W_cruise     = compute_T_W_cruise(K1,K2,W_S,alt,Vcruise)
    
    # ----------------------------------------------------------------
    # Cruise: Max cruise speed (Ps=0)
    # ----------------------------------------------------------------    
    alt = 7620 * Units.meter
    Vcruise = 214 * Units.mph    
    T_W_max_cruise = compute_T_W_max_cruise(K1,K2,W_S,alt,Vcruise)
    
    # ----------------------------------------------------------------
    # Stall Speed:
    # ----------------------------------------------------------------     
    alt = 0. * Units.meter
    Vstall = 70.2 * Units.mph
    T_W_stall = compute_T_W_stall(K1,K2,W_S,alt,Vstall)

    # ----------------------------------------------------------------
    # Rate of Climb:
    # ---------------------------------------------------------------- 
    alt = 0. * Units.meter
    V_freestream = 150. * Units.mph
    T_W_climb = compute_T_W_climb(K1,K2,W_S,alt,V_freestream)


    # ----------------------------------------------------------------
    # Constant velocity Turn:
    # ---------------------------------------------------------------- 
    alt = 6000. * Units.meter
    V_freestream = 190. * Units.mph
    T_W_turn = compute_T_W_turn(K1,K2,W_S,alt,V_freestream)
    
    # ----------------------------------------------------------------
    # Takeoff distance:
    # ---------------------------------------------------------------- 
    alt = 0. * Units.meter
    Vlo = 60. * Units.mph # Lift-off speed, > stall
    T_W_takeoff = compute_T_W_takeoff(K1,K2,W_S,alt,Vlo)
    
    # ----------------------------------------------------------------
    # Max Landing Speed:
    # ---------------------------------------------------------------- 
    alt = 0. * Units.meter
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    rho = atmo_data.density[0,0]    
    CLmax = 2.5 
    V_max_landing = 92 * Units.mph
    W_S_landing_distance = ((V_max_landing**2/2) * rho * CLmax) # [N/m^2]
    
    
    # ----------------------------------------------------------------    
    # Cessna 208b Design Point:
    # ----------------------------------------------------------------
    mtow     = 3985. * Units.kilogram * 9.81 # [N]
    T        = 7500.  # sea-level static thrust [N]
    Sref     = 279. * Units.feet**2
    W_S_208b = mtow / Sref #153.74

    # ----------------------------------------------------------------    
    # Plot constraint diagram:
    # ----------------------------------------------------------------
    fig, ax1 = plt.subplots()
    
    #W_S = W_S/9.81 # wing loading [kg/m^2]
    plt1a = ax1.plot(
        W_S,
        T_W_climb,
        "-",
        color="tab:red",
        label="Climb",
    )
    plt1b = ax1.plot(
        np.ones_like(W_S)*W_S_landing_distance,
        np.linspace(0,2,len(W_S)),
        "-",
        color="tab:orange",
        label="Landing Distance",
    )
    plt1c = ax1.plot(
        W_S,
        T_W_cruise,
        "-",
        color="tab:green",
        label="Cruise",
    )
    plt1d = ax1.plot(W_S,
        T_W_stall,
        "-",
        color="tab:purple",
        label="Stall",
    )    
    
    plt1f = ax1.plot(
        W_S,
        T_W_max_cruise,
        "-",
        color="tab:blue",
        label="Max Cruise Speed",
    )    
    plt1g = ax1.plot(
        W_S,
        T_W_turn,
        "-",
        color="tab:brown",
        label="Turning",
    )        
    plt1h = ax1.plot(
        W_S,
        T_W_takeoff,
        "-",
        color="tab:gray",
        label="Takeoff",
    )     
    
    plot1e = ax1.plot(W_S_208b, (T/mtow), "o", label = 'Cessna 208B') # sea-level static thrust : MTOW
    plot1i = ax1.plot(2026, .152, "ro", label='Optimum')
    ax1.set_xlabel("Wing Loading ($N/m^2)$")
    ax1.set_ylabel("Thrust to Weight Ratio")
    ax1.set_title("Constraint Diagram")
    ax1.set_ylim(0, 0.5)
    plt.grid()
    plt.legend(loc='upper right')
    plt.show()    
    
    return 

def compute_T_W_cruise(K1,K2,W_S,alt,Vcruise):
    
    # Atmospheric conditions for given altitude:
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    
    rho = atmo_data.density[0,0]
    q = 0.5 * rho * Vcruise**2
    
    CD0 = 0.015
    
    # Compute T/W 
    T_W_cruise = q*CD0/W_S + K1*W_S/q
    
    return T_W_cruise


def compute_T_W_max_cruise(K1,K2,W_S,alt,Vcruise):
    # ----------------------------------------------------------------
    # Cruise: Constant altitude/speed, (Ps=0)
    # ----------------------------------------------------------------   
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    
    rho = atmo_data.density[0,0]
    T   = atmo_data.temperature[0,0]
    a   = atmo_data.speed_of_sound[0,0]     
    
    alpha = compute_lapse_rate(alt,a, T, Vcruise)
    q = 0.5 * rho * Vcruise**2
    n = 1 # load factor
    beta = 0.98 # fuel/payload
    
    CD0 = 0.019
    CDR = 0 # excrescence drag
    
    T_W_1 = ((K1*(n*beta)**2)/(alpha*q))*W_S
    T_W_2 = beta*K2*n/alpha # Linear drag component
    T_W_3 = (q/(alpha*W_S))*(CD0+CDR)  
    
    T_W_max_cruise = T_W_1 + T_W_2 + T_W_3
    
    #T_W_max_cruise2 = q*CD0/W_S + K1*W_S/q
    
    return T_W_max_cruise

def compute_T_W_stall(K1,K2,W_S,alt,V):
 
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    
    rho = atmo_data.density[0,0]
    T   = atmo_data.temperature[0,0]
    a   = atmo_data.speed_of_sound[0,0]    
    
    alpha = compute_lapse_rate(alt,a, T, V)
    q = 0.5 * rho * V**2
    n = 1 # load factor
    beta = 1 # fuel/payload
    
    CD0 = 0.03
    CDR = 0 # excrescence drag
    
    T_W_1 = ((K1*(n*beta)**2)/(alpha*q))*W_S
    T_W_2 = beta*K2*n/alpha # Linear drag component
    T_W_3 = (q/(alpha*W_S))*((CD0+CDR)/q) 
    
    T_W_stall = T_W_1 + T_W_2 + T_W_3
    
    return T_W_stall


def compute_T_W_climb(K1,K2,W_S,alt,V_freestream):
  
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    rho = atmo_data.density[0,0]
    
    Vv = 14.023 * Units.mph # 1234. ft/min
    q = 0.5 * rho * V_freestream**2
    CD0 = 0.017
    
    T_W_climb = (Vv/V_freestream) + (q*CD0/W_S) + (K1/q)*W_S
    
    return T_W_climb    

def compute_T_W_turn(K1,K2,W_S,alt,V_freestream):
  
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    rho = atmo_data.density[0,0]
  
    q   = 0.5 * rho * V_freestream**2
    phi = 12 * Units.deg # max bank angle
    n   = 1/np.cos(phi) # load factor
    
    CD0 = 0.015
    
    T_W_turn = q*((CD0/W_S) + K1*(n/q)**2*W_S)
    
    return T_W_turn   



def compute_T_W_takeoff(K1,K2,W_S,alt,Vlo):
  
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(altitude = alt) 
    rho = atmo_data.density[0,0]
    
    CLto = 0.80 # Lift coefficient during takeoff run
    CDto = 0.06 # Drag coefficient during takeoff run
    q    = 0.5 * rho * (Vlo/np.sqrt(2))**2
    Sg   = 354 * Units.meter # Takeoff ground roll
    g    = 9.81 
    mu   = 0.04 # ground friction constant

    T_W_takeoff = Vlo**2/(2*g*Sg) + q*CDto/W_S + mu*(1-q*CLto/W_S)

    return T_W_takeoff

def compute_lapse_rate(alt,a, T, V):
    delta = (1-0.00000687535*alt)**5.2561
    T_deg_F = (T - 273.15)*( 9/5) + 32
    theta = (T_deg_F+459.67)/518.69
    sigma = delta/theta
    M = V/a
    alpha = (0.568+0.25*(1.2-M)**3)*sigma**0.6
    
    return alpha


if __name__ == "__main__":
    main()
    plt.show()

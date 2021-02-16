"""
A simple aircraft model designed to be extended as needed

    Nick Goodson
    Feb 2021
"""


class Aircraft:

    # =====================
    # Default parameters
    # =====================

    # Mass
    dry_mass = 3000                 # [kg]
    battery_mass = 1000             # [kg]

    # Energy
    energy_density = 220 * 3600     # [J/kg]
    packing_factor = 0.8
    capcity_fade = 0.15             # fraction of total capacity
    naccessible_fraction = 0.08     # fraction of total capacity
    reserve_fraction = 0.2          # fraction of useable capacity
    cruise_fraction = 0.6           # fraction of useable capacity

    # Aero Basic
    LD_ratio = 11

    # Aero Detailed
    wingspan = 15.87                 # [m]
    wing_area = 25.96                # [m^2]
    wing_efficiency = 0.8            # Oswald efficiency factor
    drag_coeff_parasitic = 0.034     # estimate for small AoA

    # performance
    cruise_speed = 95                # [m/s]
    cruise_altitude = 3200           # [m]
    climb_rate = 6.3                 # [m/s]
    acceleration = 0.2               # [m/s^2]

    # Propulsion
    engine_max_power = 503e3         # [W]
    prop_diameter = 2.69             # [m] guesstimate
    drivetrain_efficiency = 0.95

    def __init__(self, cfg=None):
        """
        @param cfg, a dictionary of parameters to set

        """
        if cfg:
            for key, val in cfg.items():
                setattr(self, key, val)
            print("\nAircraft initialized")
        else:
            print("\nDefault aircraft initialized")

    @property
    def total_mass(self):
        _total_mass = self.dry_mass + self.battery_mass
        return _total_mass

    @property
    def total_energy(self):
        _total_energy = self.battery_mass * self.energy_density * self.packing_factor # [J]
        return _total_energy

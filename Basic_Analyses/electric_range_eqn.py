"""
Lowest order analysis, computes cruise range based on basic vehicle properties

    Nick Goodson
    Feb 2021
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import pdb

# ==============
# Constants
# ==============
gravity = 9.81  # [m/s^2]
Wh_to_J = 3600
kmh_to_ms = 1 / 3.6


def exampleAnalysis():
    """
    An example analysis showing how the inputs required
    and how to run a fixed point or a parametric analysis

    """

    # =====================
    # Define Aircraft
    # =====================
    cfg = {
        "gross_mass":              3000,           # [kg]
        "battery_mass_fraction":   0.4,            # battery mass / gross mass
        "energy_density":          220 * Wh_to_J,  # [J/kg]
        "packing_factor":          0.8,
        "capcity_fade":            0.15,           # fraction of total capacity
        "inaccessible_fraction":   0.08,           # fraction of total capacity
        "reserve_fraction":        0.2,            # fraction of useable capacity
        "cruise_fraction":         0.6,            # fraction of useable capacity
        "drivetrain_efficiency":    0.9,
        "LD_ratio":                11
    }


    # ======================
    # Fixed Point Analysis
    # ======================
    analysis = RangeAnalysis(cfg)

    performance_new = analysis.electricCruiseRange()
    performance_eol = analysis.electricCruiseRange(end_of_life=True)
    print(f"\nCruise range (new): {performance_new['cruise_range'] / 1000 :.1f} km")
    print(f"Cruise range (end of life): {performance_eol['cruise_range'] / 1000:.1f} km")


    # =====================
    # Parameteric Analyses
    # =====================

    # 1D parametric study
    LD_s = np.arange(8, 40, 1)
    analysis.parametricStudy(["LD_ratio"], [LD_s])

    # 2D parametric study
    E_densities = np.arange(220, 800, 10) * Wh_to_J  # [J/kg]
    display_units = [1, 1.0 / Wh_to_J]
    analysis.parametricStudy(["LD_ratio", "energy_density"],
                             [LD_s, E_densities],
                              display_units=display_units,
                              end_of_life=True)


class RangeAnalysis:

    def __init__(self, aircraft):
        self.aircraft = aircraft

    def electricCruiseRange(self, end_of_life=False):
        """
        Computes aircarft cruise range using the 'electric range formula'
        
        @param end_of_life (bool): when true, battery capacity fade is included
        @return outputs (dict): battery_mass
                                total_energy
                                useable_energy
                                reserve_energy
                                cruise_energy
                                cruise_range

        """
        cfg = self.aircraft
        outputs = {}
        outputs['battery_mass'] = cfg['battery_mass_fraction'] * cfg['gross_mass']
        outputs['total_energy'] = outputs['battery_mass'] * cfg['energy_density'] * cfg['packing_factor']
        if end_of_life:
            outputs['total_energy'] *= (1 - cfg['capcity_fade'])
        outputs['useable_energy'] = outputs['total_energy'] * (1 - cfg['inaccessible_fraction'])
        outputs['reserve_energy'] = outputs['useable_energy'] * cfg['reserve_fraction']
        outputs['cruise_energy'] = outputs['useable_energy'] * cfg['cruise_fraction']
        outputs['cruise_range'] = cfg['drivetrain_efficiency'] * cfg['LD_ratio'] * outputs['cruise_energy'] /\
                                                                             (gravity * cfg['gross_mass'])

        return outputs

    def parametricStudy(self, parameters, values, display_units=None, end_of_life=False):
        """
        Perform a parametric study of ONE or TWO variables taking on specified values

        @param parameters, the names of the parameters to study (tuple of strings)
        @param values, the values to evaluate the parameters at (tuple of iterables)
        @param display_units, unit conversions from SI to the desired display unit (tuple)
        
        @return flight_ranges, The flight range at each design point (ndarray)

        """
        if len(parameters) > 2:
            raise ArgumentError("parametricStudy handles a maximum of two parameters")
        elif len(parameters) != len(values):
            raise ArgumentError("number of value sets must equal number of parameters")

        if display_units:
            if len(display_units) != len(parameters):
                raise ArgumentError("number of display_units specified must match number of parameters")
        else:
            display_units = [1] * len(parameters)

        if len(parameters) == 1:
            flight_ranges = self._parametricStudy1D(parameters[0], values[0], display_units[0], end_of_life)
        else:
            flight_ranges = self._parametricStudy2D(parameters, values, display_units, end_of_life)

        return flight_ranges

    def _parametricStudy1D(self, parameter, values, display_units, end_of_life):
        """
        Perform a parametric analysis of a single variable taking on specified values

        """
        flight_ranges = np.zeros(len(values))
        for i, val in enumerate(values):
            try:
                self.aircraft[parameter] = val
            except KeyError:
                print(f"Error: {parameter} is not a key of {self.aircraft.__name__}")
                return
            outputs = self.electricCruiseRange(end_of_life=end_of_life)
            flight_ranges[i] = outputs['cruise_range']
        self._visualize1D(flight_ranges, parameter, values, display_units)

        return flight_ranges

    def _parametricStudy2D(self, parameters, values, display_units, end_of_life):
        """
        Perform a parameteric analysis of two variables taking on specified values

        """
        flight_ranges = np.zeros((len(values[1]), len(values[0])))
        for i, valA in enumerate(values[0]):
            try:
                self.aircraft[parameters[0]] = valA
            except KeyError:
                print(f"Error: {parameter[0]} is not a key of {self.aircraft.__name__}")
                return

            for j, valB in enumerate(values[1]):
                try:
                    self.aircraft[parameters[1]] = valB
                except KeyError:
                    print(f"Error: {parameters[1]} is not a key of {self.aircraft.__name__}")
                    return
                outputs = self.electricCruiseRange(end_of_life=end_of_life)
                flight_ranges[j, i] = outputs['cruise_range']
        self._visualize2D(flight_ranges, parameters, values, display_units)

        return flight_ranges

    @staticmethod
    def _visualize1D(flight_ranges, parameter, values, display_units):
        """
        Make a standard plot of 1D parametric sim results

        """
        range_kms = flight_ranges / 1000
        values *= display_units
        fig, ax = plt.subplots(figsize=(8,10))
        ax.plot(values, range_kms, '-k')
        ax.grid(True)
        ax.set_xlabel(parameter)
        ax.set_ylabel("Cruise Range [km]")
        plt.show()

    @staticmethod
    def _visualize2D(flight_ranges, parameters, values, display_units):
        """
        Make some standard plots of 2D parametric sim results

        """
        range_kms = flight_ranges / 1000
        for i in range(2): values[i] = [val * display_units[i] for val in values[i]]

        # contours
        fig1, ax1 = plt.subplots(figsize=(8,10))
        CF1 = ax1.contourf(values[0], values[1], range_kms)
        ax1.set_xlabel(parameters[0])
        ax1.set_ylabel(parameters[1])
        cbar = fig1.colorbar(CF1)
        cbar.ax.set_ylabel("Range [km]")

        # 3D surface
        fig2 = plt.figure(figsize=(8,10))
        ax2 = fig2.gca(projection='3d')
        VA, VB = np.meshgrid(values[0], values[1], indexing='xy') 
        surf = ax2.plot_surface(VA, VB, range_kms, 
                            cmap=cm.coolwarm, linewidth=0, antialiased=False)
        ax2.set_xlabel(parameters[0])
        ax2.set_ylabel(parameters[1])
        ax2.set_zlabel("Cruise Range [km]")
        plt.show()


class ArgumentError(ValueError):
    """Raise when incorrect arguments passed to parameteric analysis"""


def main():
    """

    """
    exampleAnalysis()


if __name__ == "__main__":
    main()
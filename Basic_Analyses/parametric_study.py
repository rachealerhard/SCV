"""
A tool for performing quick parametric analyses on simulations

    Nick Goodson
    Feb 2021
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import copy
import pdb


class ArgumentError(ValueError):
    """Raise when incorrect arguments passed to parameteric analysis"""


class ParametricAnalysis:
    """
    A parametric analysis and visualization tool
    """

    def __init__(self, evaluator_func, cfg):
        self.evaluator_func = evaluator_func
        self.cfg = cfg

    def parametricStudy(self, parameters, values, display_units=None, *args, **kwargs):
        """
        Perform a parametric study of ONE or TWO variables taking on specified values

        @param parameters, the names of the parameters to study (tuple of strings)
        @param values, the values to evaluate the parameters at (tuple of iterables)
        @param display_units, unit conversions from SI to the desired display unit (tuple)
        @param *args, **kwargs, additional inputs to the evaluator function

        @return results, The ouput from the evaluator function at each design point (ndarray)

        """
        if len(parameters) > 2:
            raise ArgumentError("parametricStudy handles a maximum of two parameters")
        elif len(parameters) != len(values):
            raise ArgumentError("number of value sets must equal number of parameters")

        if display_units:
            if len(display_units) != (len(parameters) + 1):
                raise ArgumentError("number of display_units specified must == 1 + number of parameters")
        else:
            display_units = [1] * (len(parameters) + 1)

        if len(parameters) == 1:
            results = self._parametricStudy1D(parameters[0], values[0], display_units, *args, **kwargs)
        else:
            results = self._parametricStudy2D(parameters, values, display_units, *args, **kwargs)

        return results

    def _parametricStudy1D(self, parameter, values, display_units, *args, **kwargs):
        """
        Perform a parametric analysis of a single variable taking on specified values

        """
        cfg = copy.deepcopy(self.cfg)
        outputs = np.zeros(len(values))
        for i, val in enumerate(values):
            try:
                setattr(cfg, parameter, val)
            except AttributeError:
                print(f"Error: {parameter} is not a key of {type(self.cfg).__name__}")
                return
            outputs[i] = self.evaluator_func(cfg, *args, **kwargs)
        self._visualize1D(outputs, parameter, values, display_units)

        return outputs

    def _parametricStudy2D(self, parameters, values, display_units, *args, **kwargs):
        """
        Perform a parameteric analysis of two variables taking on specified values

        """
        cfg = copy.deepcopy(self.cfg)
        outputs = np.zeros((len(values[1]), len(values[0])))
        for i, valA in enumerate(values[0]):
            try:
                setattr(cfg, parameters[0], valA)
            except AttributeError:
                print(f"Error: {parameters[0]} is not a key of {type(self.cfg).__name__}")
                return

            for j, valB in enumerate(values[1]):
                try:
                    setattr(cfg, parameters[1], valB)
                except AttributeError:
                    print(f"Error: {parameters[1]} is not a key of {type(self.cfg).__name__}")
                    return
                outputs[j, i] = self.evaluator_func(cfg, *args, **kwargs)
        self._visualize2D(outputs, parameters, values, display_units)

        return outputs

    def _visualize1D(self, outputs, parameter, values, display_units):
        """
        Make a standard plot of 1D parametric sim results

        """
        display_output = outputs * display_units[-1]
        values *= display_units[0]
        fig, ax = plt.subplots(figsize=(8,10))
        ax.plot(values, display_output, '-k')
        ax.grid(True)
        ax.set_xlabel(parameter)
        ax.set_ylabel(self.evaluator_func.__name__)
        plt.show()

    def _visualize2D(self, outputs, parameters, values, display_units):
        """
        Make some standard plots of 2D parametric sim results

        """
        display_output = outputs * display_units[-1]
        for i in range(2): values[i] = [val * display_units[i] for val in values[i]]

        # contours
        fig1, ax1 = plt.subplots(figsize=(8,10))
        CF1 = ax1.contourf(values[0], values[1], display_output)
        ax1.set_xlabel(parameters[0])
        ax1.set_ylabel(parameters[1])
        cbar = fig1.colorbar(CF1)
        cbar.ax.set_ylabel(self.evaluator_func.__name__)

        # 3D surface
        fig2 = plt.figure(figsize=(8,10))
        ax2 = fig2.gca(projection='3d')
        VA, VB = np.meshgrid(values[0], values[1], indexing='xy') 
        surf = ax2.plot_surface(VA, VB, display_output, 
                            cmap=cm.coolwarm, linewidth=0, antialiased=False)
        ax2.set_xlabel(parameters[0])
        ax2.set_ylabel(parameters[1])
        ax2.set_zlabel(self.evaluator_func.__name__)
        plt.show()

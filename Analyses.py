# Analyses.py
#
# Created:  Mar. 2016, M. Vegh
# Modified:

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE

import numpy as np
from SUAVE.Core import Units
# from Optimize import AVL_analysis
# from supporting.empty_saga import empty
from SUAVE.Methods.Weights.Correlations.General_Aviation import empty
# from supporting.stability_saga import Fidelity_Zero


# ----------------------------------------------------------------------
#   Setup Analyses
# ----------------------------------------------------------------------

def setup(configs):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag, config in configs.items():
        analysis = base(config)
        analyses[tag] = analysis

    # adjust analyses for configs

    # takeoff_analysis
    analyses.takeoff.aerodynamics.settings.drag_coefficient_increment = 0.0000

    # landing analysis
    aerodynamics = analyses.landing.aerodynamics

    return analyses


# ----------------------------------------------------------------------
#   Define Base Analysis
# ----------------------------------------------------------------------

def base(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    # weights.settings.empty_weight_method = empty
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # if not AVL_analysis:  # Run zero-fidelity method
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    # aerodynamics.process.compute.lift.inviscid_wings.training.angle_of_attack = np.array([[-5., 0.0, 5.0, 10.0, 75.]]).T * Units.deg
    # aerodynamics.process.compute.lift.inviscid_wings.training.Mach            = np.array([[0.0, 0.2, 0.5, 0.70, 0.80, 0.9, 1.3, 1.35, 1.5, 2.0]]).T
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # # AVL-based analysis
    # else:
    #     aerodynamics = SUAVE.Analyses.Aerodynamics.AVL()
    #     aerodynamics.geometry = vehicle
    #     aerodynamics.features = vehicle
    #     analyses.append(aerodynamics)

    # # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # #  Noise Analysis
    # noise = SUAVE.Analyses.Noise.Fidelity_One()
    # noise.geometry = vehicle
    # analyses.append(noise)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors  # what is called throughout the mission (at every time step))
    analyses.append(energy)
    #

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976(temperature_deviation=0.0)
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    # done!
    return analyses

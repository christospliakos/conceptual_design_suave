# Optimize.py
# Created:  Feb 2016, M. Vegh
# Modified:

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
import matplotlib.pyplot as plt
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Optimization import Nexus, carpet_plot
from SUAVE.Plots.Mission_Plots import *

import Analyses_Pliakos as Analyses
import Missions_Pliakos as Mission
import Plot_Mission
import Procedure_Pliakos as Procedure
import Vehicle_Pliakos as Vehicles

# ----------------------------------------------------------------------
#   Run the whole thing
# ----------------------------------------------------------------------

AVL_analysis = False  # AVL Analysis switch


def main():
    print("SUAVE initialized...\n")
    problem = setup()  # "problem" is a nexus

    # output = problem.objective()  # uncomment this line when using the default inputs
    # variable_sweep(problem)  # uncomment this to view some contours of the problem
    output = scipy_setup.SciPy_Solve(problem, solver='SLSQP')  # uncomment this to optimize the values
    print(output)

    # print('constraints=', problem.all_constraints())


    results = problem.results
    # Plot Flight Conditions
    # plot_flight_conditions(results)
    #
    # # Plot Aerodynamic Forces
    # plot_aerodynamic_forces(results)
    #
    # # Plot Aerodynamic Coefficients
    # plot_aerodynamic_coefficients(results)
    #
    # # Drag Components
    # plot_drag_components(results)
    #
    # # Plot Altitude, sfc, vehicle weight
    # plot_altitude_sfc_weight(results)
    #
    # # Plot Velocities
    # plot_aircraft_velocities(results)

    # Plot_Mission.plot_mission(problem.results, show=False)

    return


# ----------------------------------------------------------------------
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------

def setup():
    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    problem.inputs = np.array([
        # Variable inputs
        ['wing_area', 30, 20., 80., 30, 1*Units.meter ** 2],
        ['h_tail_area', 10, 2, 30, 1, 1 * Units.meter ** 2],
        ['v_tail_area', 10, 2, 40, 1, 1 * Units.meter ** 2],
        ['AR', 8, 6, 12., 1, 1 * Units.less],
        ['MTOW', 8000, 6000, 12000, 1, 1*Units.kg],
        ['design_thrust', 1000, 500, 5000, 1, 1*Units.N],

        # ['payload', 1360, (1360, 35e3), 30e3, Units.kg],
        # ['fus_length', 20, 14, 25, 1, 1*Units.meter],
        # ['oswald', 0.8, 0.75, 0.9, 1, 1*Units.less],
        # ['bypass', 7.5, 6, 12, 1, 1*Units.less],

    ], dtype=object)

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    problem.objective = np.array([
         [ 'Nothing', 1. , 1*Units.kg],
    ],dtype=object)

    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------

    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([

        ['max_throttle', '>', 0., 1., 1*Units.less],
        ['min_throttle', '>', 0., 1., 1 * Units.less],
        ['cruise_distance', '>', 1000, 1., 1*Units.km],
        # ['main_mission_time', '<', 11.1, 10, Units.h],
        # ['stall_speed', '>', 0.397, 0.01, 1*Units.less],  # Mach speed
        # ['take_off_field_length', '<', 810., 810, Units.m],
        # ['landing_field_length', '<', 810., 810, Units.m],
        ['payload', '>', 1300., 10, 1*Units.kg],
        # ['clmax', '<', 1.1, 1, Units.less],
    ], dtype=object)

    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------

    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        ['wing_area', 'vehicle_configurations.*.wings.main_wing.areas.reference'],  # 'vehicle_configurations.*.reference_area'
        ['MTOW', 'vehicle_configurations.*.mass_properties.max_takeoff'],  # 'vehicle_configurations.*.mass_properties.takeoff',
        ['design_thrust', 'vehicle_configurations.*.propulsors.turbofan.thrust.total_design'],
        ['AR', 'vehicle_configurations.*.wings.main_wing.aspect_ratio'],
        ['h_tail_area', 'vehicle_configurations.*.wings.horizontal_stabilizer.areas.reference'],
        ['v_tail_area', 'vehicle_configurations.*.wings.vertical_stabilizer.areas.reference'],
        ['fus_length', 'vehicle_configurations.*.fuselages.fuselage.lengths.total'],
        ['payload', 'summary.max_payload'],  #, 'vehicle_configurations.*.mass_properties.payload']
        ['cruise_distance', 'summary.mission_range'],
        ['min_throttle', 'summary.min_throttle'],
        ['max_throttle', 'summary.max_throttle'],
        ['main_mission_time', 'summary.main_mission_time'],
        ['mission_range', 'summary.mission_range'],
        ['clmax', 'summary.clmax'],
        ['design_range_fuel_margin', 'summary.max_zero_fuel_margin'],
        ['take_off_field_length', 'summary.takeoff_field_length'],
        ['landing_field_length', 'summary.landing_field_length'],
        # ['stall_speed', 'missions.base.segments.descent.mach_end'],
        ['oswald', 'vehicle_configurations.*.wings.main_wing.span_efficiency'],
        ['bypass', 'vehicle_configurations.*.propulsors.turbofan.bypass_ratio'],
        ['Nothing', 'summary.nothing'],

    ]

    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()

    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)

    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Mission.setup(nexus.analyses)

    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------
    nexus.procedure = Procedure.setup()

    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------
    nexus.summary = Data()

    return nexus


def variable_sweep(problem, color_label, bar_label, xlabel, ylabel, title):
    number_of_points = 5
    outputs = carpet_plot(problem, number_of_points, 0, 0)  # run carpet plot, suppressing default plots
    inputs = outputs.inputs
    objective = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS = plt.contourf(inputs[0, :], inputs[1, :], objective, 20, linewidths=2, cmap='hot')
    cbar = plt.colorbar(CS)
    cbar.ax.set_ylabel(color_label)
    # cbar.ax.set_ylabel('fuel burn (kg)')

    if bar_label != "unknown":
        CS_const = plt.contour(inputs[0, :], inputs[1, :], constraints[0, :, :])
        plt.clabel(CS_const, inline=1, fontsize=10)
        cbar = plt.colorbar(CS_const)
        # cbar.ax.set_ylabel('fuel margin')
        cbar.ax.set_ylabel(bar_label)

    # plt.xlabel('wing area (m^2)')
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    # plt.ylabel('cruise_speed (km)')

    '''
    #now plot optimization path (note that these data points were post-processed into a plottable format)
    wing_1  = [95          ,	95.00000149 ,	95          ,	95          ,	95.00000149 ,	95          ,	95          ,	95.00000149 ,	95          ,	106.674165  ,	106.6741665 ,	106.674165  ,	106.674165  ,	106.6741665 ,	106.674165  ,	106.674165  ,	106.6741665 ,	106.674165  ,	105.6274294 ,	105.6274309 ,	105.6274294 ,	105.6274294 ,	105.6274309 ,	105.6274294 ,	105.6274294 ,	105.6274309 ,	105.6274294 ,	106.9084316 ,	106.9084331 ,	106.9084316 ,	106.9084316 ,	106.9084331 ,	106.9084316 ,	106.9084316 ,	106.9084331 ,	106.9084316 ,	110.520489  ,	110.5204905 ,	110.520489  ,	110.520489  ,	110.5204905 ,	110.520489  ,	110.520489  ,	110.5204905 ,	110.520489  ,	113.2166831 ,	113.2166845 ,	113.2166831 ,	113.2166831 ,	113.2166845 ,	113.2166831 ,	113.2166831 ,	113.2166845 ,	113.2166831 ,	114.1649262 ,	114.1649277 ,	114.1649262 ,	114.1649262 ,	114.1649277 ,	114.1649262 ,	114.1649262 ,	114.1649277 ,	114.1649262 ,	114.2149828]
    alt_1   = [11.0              ,	11.0              ,	11.000000149011612,	11.0              ,	11.0              ,	11.000000149011612,	11.0              ,	11.0              ,	11.000000149011612,	9.540665954351425 ,	9.540665954351425 ,	9.540666103363037 ,	9.540665954351425 ,	9.540665954351425 ,	9.540666103363037 ,	9.540665954351425 ,	9.540665954351425 ,	9.540666103363037 ,	10.023015652305284,	10.023015652305284,	10.023015801316896,	10.023015652305284,	10.023015652305284,	10.023015801316896,	10.023015652305284,	10.023015652305284,	10.023015801316896,	10.190994033521863,	10.190994033521863,	10.190994182533474,	10.190994033521863,	10.190994033521863,	10.190994182533474,	10.190994033521863,	10.190994033521863,	10.190994182533474,	10.440582829327589,	10.440582829327589,	10.4405829783392  ,	10.440582829327589,	10.440582829327589,	10.4405829783392  ,	10.440582829327589,	10.440582829327589,	10.4405829783392  ,	10.536514606250261,	10.536514606250261,	10.536514755261873,	10.536514606250261,	10.536514606250261,	10.536514755261873,	10.536514606250261,	10.536514606250261,	10.536514755261873,	10.535957839878783,	10.535957839878783,	10.535957988890395,	10.535957839878783,	10.535957839878783,	10.535957988890395,	10.535957839878783,	10.535957839878783,	10.535957988890395,	10.52829047]
    wing_2  = [128        ,	128.0000015,	128        ,	128        ,	128.0000015,	128        ,	128        ,	128.0000015,	128        ,	130        ,	130.0000015,	130        ,	130        ,	130.0000015,	130        ,	130        ,	130.0000015,	130        ,	122.9564124,	122.9564139,	122.9564124,	122.9564124,	122.9564139,	122.9564124,	122.9564124,	122.9564139,	122.9564124,	116.5744347,	116.5744362,	116.5744347,	116.5744347,	116.5744362,	116.5744347,	116.5744347,	116.5744362,	116.5744347,	116.3530891,	116.3530906,	116.3530891,	116.3530891,	116.3530906,	116.3530891,	116.3530891,	116.3530906,	116.3530891]
    alt_2   = [13.8,	13.799999999999999,	13.80000014901161,	13.799999999999999,	13.799999999999999,	13.80000014901161,	13.799999999999999,	13.799999999999999,	13.80000014901161,	11.302562430674953,	11.302562430674953,	11.302562579686565,	11.302562430674953,	11.302562430674953,	11.302562579686565,	11.302562430674953,	11.302562430674953,	11.302562579686565,	11.158808932491421,	11.158808932491421,	11.158809081503033,	11.158808932491421,	11.158808932491421,	11.158809081503033,	11.158808932491421,	11.158808932491421,	11.158809081503033,	11.412913394878741,	11.412913394878741,	11.412913543890353,	11.412913394878741,	11.412913394878741,	11.412913543890353,	11.412913394878741,	11.412913394878741,	11.412913543890353,	11.402627869388722,	11.402627869388722,	11.402628018400334,	11.402627869388722,	11.402627869388722,	11.402628018400334,	11.402627869388722,	11.402627869388722,	11.402628018400334]


    opt_1   = plt.plot(wing_1, alt_1, label='optimization path 1')
    init_1  = plt.plot(wing_1[0], alt_1[0], 'ko')
    final_1 = plt.plot(wing_1[-1], alt_1[-1], 'kx')

    opt_2   = plt.plot(wing_2, alt_2, 'k--', label='optimization path 2')
    init_2  = plt.plot(wing_2[0], alt_2[0], 'ko', label= 'initial points')
    final_2 = plt.plot(wing_2[-1], alt_2[-1], 'kx', label= 'final points')
    '''
    plt.legend(loc='upper left')
    plt.savefig(title + ".eps")
    plt.show()

    return


if __name__ == '__main__':
    main()

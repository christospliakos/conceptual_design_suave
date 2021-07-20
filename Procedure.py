# Procedure.py
#
# Created:  Mar 2016, M. Vegh
# Modified:

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
import numpy as np
import os
from SUAVE.Analyses.Process import Process
from SUAVE.Core import Units, Data
from SUAVE.Input_Output.Results import print_parasite_drag, \
    print_compress_drag, \
    print_weight_breakdown
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift.compute_max_lift_coeff import compute_max_lift_coeff
# from SUAVE.Methods.Center_of_Gravity.compute_aircraft_center_of_gravity import compute_aircraft_center_of_gravity
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion.compute_turbofan_geometry import \
    compute_turbofan_geometry
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform
from SUAVE.Methods.Noise.Fidelity_One.Airframe import noise_airframe_Fink
from SUAVE.Methods.Noise.Fidelity_One.Engine import noise_SAE
from SUAVE.Methods.Performance import estimate_landing_field_length
from SUAVE.Methods.Performance import estimate_take_off_field_length
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Optimization import write_optimization_outputs

from supporting.print_engine_data import print_engine_data
from supporting.print_mission_breakdown import print_mission_breakdown

numpy_export = False


# ---------------------------------------------------------------------
#   Setup
# ----------------------------------------------------------------------

output_folder = 'Airbulance'

def setup():
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------

    # size the base config
    # procedure = Data()
    procedure = Process()
    procedure.simple_sizing                 = simple_sizing

    # find the weights
    procedure.weights                       = weight

    # finalizes the data dependencies
    procedure.finalize                      = finalize

    # performance studies
    procedure.missions                      = Process()
    procedure.missions.design_mission       = design_mission

    # Field lengths
    # procedure.takeoff_field_length = takeoff_field_length
    # procedure.landing_field_length = landing_field_length

    # post process the results
    procedure.post_process                  = post_process

    # done!
    return procedure


# ----------------------------------------------------------------------
#   Target Range Function
# ----------------------------------------------------------------------

def find_target_range(nexus, mission):
    segments = mission.segments

    cruise_altitude = 10 * Units.km
    # cruise_altitude = mission.segments['climb_1'].altitude_end

    # climb_1 = segments['climb_1']

    # descent_1 = segments['descent']

    # # TODO: Check whats going  on with airspeed_end
    # x_climb_1 = climb_1.altitude_end / np.tan(np.arcsin(climb_1.climb_rate / climb_1.air_speed_end))
    #
    #
    # x_descent_1 = (climb_1.altitude_end - descent_1.altitude_end) / np.tan(
    #     np.arcsin(descent_1.descent_rate / (descent_1.mach_end * 343)))  # Converting mach_end to m/s sea level

    # The new cruise range based on the climb + descent
    cruise_range = mission.design_range # -(x_climb_1+x_descent_1)
    segments['cruise'].distance = cruise_range
    segments['cruise'].altitude = cruise_altitude

    return nexus


def takeoff_field_length(nexus):

    # import tofl analysis module
    estimate_tofl = SUAVE.Methods.Performance.estimate_take_off_field_length

    # unpack data
    results  = nexus.results
    summary  = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.takeoff
    vehicle  = nexus.vehicle_configurations.base

    # defining required data for tofl evaluation
    config.mass_properties.takeoff = nexus.summary.MTOW
    takeoff_airport                = missions.base.airport

    takeoff_field_length = estimate_tofl(config, analyses.base, takeoff_airport)

    print ('TOFL: '+str('%5.1f' % takeoff_field_length)+' m')

    # pack results
    summary.takeoff_field_length = takeoff_field_length

    return nexus


def landing_field_length(nexus):

    # import tofl analysis module
    estimate_landing = SUAVE.Methods.Performance.estimate_landing_field_length

    # unpack data
    summary = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.landing

    # defining required data for tofl evaluation
    landing_airport = missions.base.airport
    config.mass_properties.landing = nexus.summary.MTOW * 0.97174

    landing_field_length = estimate_landing(config, analyses, landing_airport)

    print ('LFL for MLW (' + str('%5.1f' % config.mass_properties.landing) + ' kg): ' +
           str('%5.1f' % landing_field_length) + ' m')

    # pack results
    summary.landing_field_length = landing_field_length

    return nexus


def evaluate_field_length(configs, analyses, mission, results):
    # unpack
    airport = mission.airport

    takeoff_config = configs.takeoff
    landing_config = configs.landing

    # evaluate

    airport.atmosphere = analyses.base.atmosphere

    TOFL = estimate_take_off_field_length(takeoff_config, analyses, airport)
    LFL = estimate_landing_field_length(landing_config, analyses, airport)  # FIXME

    # pack
    field_length = SUAVE.Core.Data()
    field_length.takeoff = TOFL[0]
    field_length.landing = LFL[0]

    print("Takeoff field length: ", TOFL[0])
    print("Landing field length: ", LFL[0])

    results.field_length = field_length
    return results


# ----------------------------------------------------------------------
#   Design Mission
# ----------------------------------------------------------------------
def design_mission(nexus):
    mission = nexus.missions.base
    mission.design_range = 1200. * Units['km']  # 1.2 * Cruise_range (requirement) for safety
    find_target_range(nexus, mission)

    results = nexus.results
    results.base = mission.evaluate()

    return nexus


# ----------------------------------------------------------------------
#   Sizing
# ----------------------------------------------------------------------

def simple_sizing(nexus):
    configs = nexus.vehicle_configurations
    base = configs.base

    # find conditions
    air_speed = nexus.missions.base.segments['cruise'].air_speed
    altitude = 10 * Units.km
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()

    # Pressurized cabin.
    freestream = atmosphere.compute_values(altitude)
    freestream0 = atmosphere.compute_values(6000. * Units.ft)  # cabin altitude / Source -> Google

    diff_pressure         = np.max(freestream0.pressure-freestream.pressure, 0)
    fuselage = base.fuselages['fuselage']
    fuselage.differential_pressure = diff_pressure

    # now size engine
    mach_number = air_speed / freestream.speed_of_sound

    # now add to freestream data object
    freestream.velocity = air_speed
    freestream.mach_number = mach_number
    freestream.gravity = 9.81

    conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()  # assign conditions in form for propulsor sizing
    conditions.freestream = freestream
    # conditions.weights.vehicle_mass_rate = -200 * Units['kg/s']

    for config in configs:
        # keeping tail volume constant with wings. Maybe later
        # config.wings.horizontal_stabilizer.areas.reference = (26.0 / 92.0) * config.wings.main_wing.areas.reference

        wing_planform(config.wings.main_wing)
        for wing in config.wings:

            wing_planform(wing)
            # wing.areas.wetted = 2.0 * wing.areas.reference
            wing.areas.exposed = 0.8 * wing.areas.wetted
            wing.areas.affected = 0.6 * wing.areas.wetted

        fuselage = config.fuselages['fuselage']
        fuselage.differential_pressure = diff_pressure

        turbofan_sizing(config.propulsors['turbofan'], mach_number, altitude)
        compute_turbofan_geometry(config.propulsors['turbofan'], conditions)  # engine_length, nacelle_diameter, Swet
        # diff the new data
        config.store_diff()

    # done!

    return nexus


# ----------------------------------------------------------------------
#   Weights
# ----------------------------------------------------------------------

def weight(nexus):
    vehicle = nexus.vehicle_configurations.base

    # for config in nexus.vehicle_configurations:
    #     config.mass_properties.takeoff       = vehicle.mass_properties.max_takeoff
    #     config.mass_properties.max_takeoff   = vehicle.mass_properties.max_takeoff

    # weight analysis
    weights = nexus.analyses.base.weights.evaluate()
    compute_component_centers_of_gravity(vehicle)

    weights = nexus.analyses.cruise.weights.evaluate()
    weights = nexus.analyses.landing.weights.evaluate()
    weights = nexus.analyses.takeoff.weights.evaluate()
    vehicle.mass_properties.breakdown = weights

    for config in nexus.vehicle_configurations:
        config.mass_properties.zero_fuel_center_of_gravity = vehicle.mass_properties.zero_fuel_center_of_gravity
        config.fuel = vehicle.fuel
        config.mass_properties.operating_empty = vehicle.mass_properties.operating_empty

    nexus.summary.MTOW = vehicle.mass_properties.max_takeoff
    nexus.summary.BOW  = vehicle.mass_properties.operating_empty

    return nexus


# ----------------------------------------------------------------------
#   Finalizing Function (make part of optimization nexus)[needs to come after simple sizing doh]
# ----------------------------------------------------------------------

def finalize(nexus):
    nexus.analyses.finalize()

    return nexus


# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------

def post_process(nexus):
    # Unpack data
    vehicle = nexus.vehicle_configurations.base
    configs = nexus.vehicle_configurations
    results = nexus.results
    summary = nexus.summary
    missions = nexus.missions

    # results = evaluate_field_length(configs, nexus.analyses, missions.base, results)
    #

    # -----------------------------------------------------------------------------------------------------------------
    # throttle in design mission
    max_throttle = 0
    min_throttle = 0
    for segment in results.base.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:, 0])
        min_segment_throttle = np.min(segment.conditions.propulsion.throttle[:, 0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
        if min_segment_throttle < min_throttle:
            min_throttle = min_segment_throttle

    summary.max_throttle = max_throttle
    summary.min_throttle = min_throttle

    # -----------------------------------------------------------------------------------------------------------------
    # Fuel margin and base fuel calculations

    # vehicle.mass_properties.operating_empty += 0e3  # FIXME hardcoded wing mass correction # area scaling?
    operating_empty = vehicle.mass_properties.operating_empty
    payload = vehicle.mass_properties.payload
    max_payload = vehicle.mass_properties.max_payload
    summary.max_payload = max_payload
    design_landing_weight = results.base.segments[-1].conditions.weights.total_mass[-1]
    design_takeoff_weight = vehicle.mass_properties.takeoff
    max_takeoff_weight = nexus.vehicle_configurations.takeoff.mass_properties.max_takeoff
    zero_fuel_weight = nexus.vehicle_configurations.takeoff.mass_properties.max_zero_fuel
    # zero_fuel_weight = payload + operating_empty

    # design mission: MTOW, PLDmax for fixed range
    # summary.base_mission_fuelburn = design_takeoff_weight - results.base.segments['descent'].conditions.weights.total_mass[-1]
    summary.fuel_margin = design_landing_weight - operating_empty - max_payload
    summary.max_zero_fuel_margin  = (design_landing_weight - zero_fuel_weight)/zero_fuel_weight

    print("zero fuel weight: ", zero_fuel_weight, "kg  i.e. (", payload, "+", operating_empty, ")")
    # print("MTOW selected: ", vehicle.mass_properties.takeoff, "kg, MTOW calculated: ",
    #       zero_fuel_weight + summary.base_mission_fuelburn)
    print("Max/Min throttle: ", summary.max_throttle, ", ", summary.min_throttle)
#    print("Take-off field length: ", summary.takeoff_field_length[0], "m")
#    print("Landing field length: ", summary.landing_field_length[0], "m")
    summary.mission_range = results.base.segments['cruise'].conditions.frames.inertial.position_vector[:, 0][-1] / 1000
    print("Mission Range (must be at least 1000km): ", summary.mission_range, " km")
    summary.total_range = results.base.segments[-1].conditions.frames.inertial.position_vector[:, 0][-1] / 1000.
    print("Total Range: ", summary.total_range, " km", "(+", summary.total_range - summary.mission_range, ")")
    # summary.main_mission_time = (results.base.segments['descent'].conditions.frames.inertial.time[-1] -
    #                              results.base.segments[0].conditions.frames.inertial.time[0])
    # summary.total_mission_time = (results.base.segments[-1].conditions.frames.inertial.time[-1] -
    #                               results.base.segments[0].conditions.frames.inertial.time[0])
    # print("Mission time: ", summary.main_mission_time[0] * Units['s'] / Units.h, "hours (main) +", \
    #     (summary.total_mission_time - summary.main_mission_time)[0] * Units['s'] / Units.h, "hours (diversion)")
    summary.nothing = 0.0
    # print('Fuel burn: ', summary.base_mission_fuelburn, " Fuel margin: ", summary.max_zero_fuel_margin)
    clmax = 0
    for segment in results.base.segments.values():
        cl = np.max(segment.conditions.aerodynamics.lift_coefficient[:, 0])
        if cl > clmax:
            clmax = cl

    summary.clmax = clmax
    print("CL_max: ", summary.clmax)

    gt_engine = nexus.vehicle_configurations.base.propulsors.turbofan
    # FIXME move that to printing/results section
    print("Turbofan thrust:", gt_engine.sealevel_static_thrust, " x ", int(
        gt_engine.number_of_engines), "engines (tot: ", gt_engine.sealevel_static_thrust * gt_engine.number_of_engines,
          " N)")
    print("Thrust required: ", gt_engine.design_thrust, "N")
    print("Estimated engine length: ", gt_engine.engine_length, ", diameter: ", gt_engine.nacelle_diameter,
          ", wetted area: ", gt_engine.areas.wetted)

    # #when you run want to output results to a file
    # filename = 'results.txt'
    # write_optimization_outputs(nexus, filename)

    return nexus

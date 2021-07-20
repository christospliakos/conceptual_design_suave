# Missions.py
#
# Created:  Mar 2016, M. Vegh
# Modified:


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units
import numpy as np


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def setup(analyses):
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    base_mission = base(analyses)
    missions.base = base_mission

    # # Takeoff Mission
    # missions.takeoff = takeoff_mission_setup(analyses)
    # # Landing mission
    # missions.landing = landing_mission_setup(analyses)

    return missions


def base(analyses):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission                             = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag                         = 'the_mission'

    # airport
    airport                             = SUAVE.Attributes.Airports.Airport()
    airport.altitude                    = 0.0 * Units.km
    airport.delta_isa                   = 0.0
    airport.atmosphere                  = SUAVE.Analyses.Atmospheric.US_Standard_1976

    mission.airport                     = airport

    # unpack Segments module
    Segments                            = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                        = Segments.Segment()
    atmosphere                          = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976(temperature_deviation=0.0)
    planet                              = SUAVE.Attributes.Planets.Earth()

    climb_throttle                      = 1.0  # Constant throttle for all climb segments
    # climb_air_speed                   = 210. * Units['m/s']

    # CLIMB PHASES START HERE:

    # ------------------------------------------------------------------
    #  Take off phase
    # ------------------------------------------------------------------

    # segment                             = Segments.Ground.Takeoff(base_segment)
    # segment.tag                         = "takeoff"
    #
    # # connect vehicle configuration
    # segment.analyses.extend(analyses.takeoff)
    #
    # # define segment attributes
    # segment.atmosphere                  = atmosphere
    # segment.planet                      = planet
    #
    # segment.velocity_start              = 0.0
    # segment.velocity_end                = 37.5 * 1.1 * Units['m/s']  # Vstall is 37.5m/s from requirements - Anderson
    #                                                                 # pp 355
    # segment.friction_coefficient        = 0.04                      # Default dry asphalt
    # segment.throttle                    = 1.0                       # Full throtle.
    # segment.ground_incline              = 0.
    #
    # # segment.process.iterate.conditions.weights = update_weights_sprayer
    # # segment.sprayer_rate = 0 * Units['kg/s']
    # # segment.aerosol_mass_initial = 0. * Units.kg
    #
    # # add to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Linear Speed, Constant Rate
    # ------------------------------------------------------------------

    # segment                             = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                         = "climb_1"
    #
    # # connect vehicle configuration
    # segment.analyses.extend(analyses.takeoff)
    #
    # # define segment attributes
    # segment.atmosphere                  = atmosphere
    # segment.planet                      = planet
    #
    # segment.altitude_start              = 0.0 * Units.km
    # segment.altitude_end                = 10.0 * Units.km
    # segment.air_speed_start             = 0 * Units['m/s']  # Starting from ground?
    # segment.air_speed_end               = 138.89 * Units['m/s']  # Cruise speed
    # segment.climb_rate                  = 10 * Units['m/s']
    # segment.throttle                    = climb_throttle
    #
    # # # segment.process.iterate.conditions.weights = update_weights_sprayer
    # # segment.sprayer_rate = 0 * Units['kg/s']
    # # segment.aerosol_mass_initial = 0. * Units.kg
    #
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  First Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------

    segment                             = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                         = "cruise"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)

    # segment attributes
    segment.atmosphere                  = atmosphere
    segment.planet                      = planet

    segment.air_speed                   = 138.89 * Units['m/s']  # Mission req
    segment.distance                    = 1000 * Units.km        # Mission req

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Linear Mach decrease, Constant Rate
    # ------------------------------------------------------------------

    # segment                             = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    # segment.tag                         = "descent"
    #
    # # connect vehicle configuration
    # segment.analyses.extend(analyses.landing)
    #
    # # segment attributes
    # segment.atmosphere                  = atmosphere
    # segment.planet                      = planet
    #
    # segment.descent_rate                = 10. * Units['m/s']  # Same as climbing rate cause why not
    # segment.altitude_end                = 0 * Units.km
    # segment.altitude_start              = 10 * Units.km
    # segment.mach_start                  = 0.464               # 500kph at 10km
    # segment.mach_end                    = 0.397               # 135 kph stall speed at sea level
    # # segment.air_speed                   = 0
    #
    # mission.append_segment(segment)

    #  ------------------------------------------------------------------
    #   Mission definition complete
    # ------------------------------------------------------------------

    return mission


def takeoff_mission_setup(analyses):
    # ------------------------------------------------------------------
    #   Initialize the Mission segment for takeoff
    # ------------------------------------------------------------------
    mission                     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag                 = 'takeoff'

    # airport
    airport                     = SUAVE.Attributes.Airports.Airport()
    airport.altitude            = 0.0 * Units.ft
    airport.delta_isa           = 0.0
    airport.atmosphere          = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    mission.airport             = airport

    # unpack Segments module
    Segments                    = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                = Segments.Segment()

    # Climb Segment: Constant throttle, constant speed
    segment                     = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag                 = "climb"
    segment.analyses.extend(analyses.takeoff)
    segment.altitude_start = 0. * Units.fts
    segment.altitude_end = 304.8 * Units.meter
    segment.air_speed = 85.4 * Units['m/s']
    segment.throttle = 1.
    segment.state.numerics.number_control_points = 10
    # segment.state.numerics.discretization_method = SUAVE.Methods.Utilities.Chebyshev.linear_data
    mission.append_segment(segment)

    # Cutback Segment: Constant speed, constant segment angle
    segment = Segments.Climb.Constant_Speed_Constant_Angle_Noise(base_segment)
    segment.tag = "cutback"
    segment.analyses.extend(analyses.takeoff)
    segment.air_speed = 85.4 * Units['m/s']
    segment.climb_angle = 2.86 * Units.degrees
    # segment.state.numerics.discretization_method = SUAVE.Methods.Utilities.Chebyshev.linear_data
    mission.append_segment(segment)

    return mission


def landing_mission_setup(analyses):
    # ------------------------------------------------------------------
    #   Initialize the Mission segment for landing
    # ------------------------------------------------------------------
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'landing'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude = 0.0 * Units.ft
    airport.delta_isa = 0.0
    airport.atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    mission.airport = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    # ------------------------------------------------------------------
    #   Descent Segment: Constant speed, constant segment angle
    # ------------------------------------------------------------------
    segment = Segments.Descent.Constant_Speed_Constant_Angle_Noise(base_segment)
    segment.tag = "descent"
    segment.analyses.extend(analyses.landing)

    segment.air_speed = 67. * Units['m/s']
    segment.descent_angle = 3.0 * Units.degrees
    # segment.state.numerics.discretization_method = SUAVE.Methods.Utilities.Chebyshev.linear_data
    mission.append_segment(segment)

    return mission


# ----------------------------------------------------------------------
#   Call Main
# ----------------------------------------------------------------------

if __name__ == '__main__':
    import vehicles
    import analyses

    vehicles = vehicles.setup()
    analyses = analyses.setup(vehicles)
    missions = setup(analyses)

    vehicles.finalize()
    analyses.finalize()
    missions.finalize()

    missions.base.evaluate()

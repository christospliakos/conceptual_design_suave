# Vehicles.py
#
# Created:  Feb. 2016, M. Vegh
# Modified:

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
import numpy as np
from SUAVE.Core import Units

from supporting.S_wetted_wing import S_wet_w, S_wet_fus
from supporting.engine import engine_caluclations
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform, horizontal_tail_planform_raymer, \
    vertical_tail_planform_raymer, fuselage_planform


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)

    return configs


def base_setup():
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Airmbulance'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    # We can select some requirements from historical data..
    vehicle.mass_properties.max_takeoff     = 8000. * Units.kg  # selected in Optimize.py and first estimates
    vehicle.mass_properties.takeoff         = 8000. * Units.kg  # selected in Optimize.py and first estimates
    vehicle.mass_properties.max_payload     = 1360 * Units.kg  # Mission Requirement (10 passengers + cargo)
    vehicle.mass_properties.payload         = 1360 * Units.kg  # Mission Requirement (10 passengers + cargo)
    vehicle.mass_properties.crew            = 6  # Mission requirement
    vehicle.mass_properties.passenger       = 10  # Mission requirement
    vehicle.mass_properties.operating_empty = 0.60 * vehicle.mass_properties.max_takeoff  # We/W0 from similar aircrafts
    vehicle.mass_properties.max_fuel        = vehicle.mass_properties.max_takeoff \
                                              - vehicle.mass_properties.operating_empty \
                                              - vehicle.mass_properties.max_payload

    # vehicle.mass_properties.operating_empty           = 65000.   # kg

    # vehicle.mass_properties.max_zero_fuel             = 105000. #60899.3  # kg
    # vehicle.mass_properties.cargo                     = 0.0 * Units.kg

    # vehicle.mass_properties.max_fuel                  = 30000.

    # vehicle.mass_properties.center_of_gravity = [18., 0, 0]
    # vehicle.mass_properties.moments_of_inertia.tensor = [[49623674.97, 0, 0], [0, 6014300.75, 0, ],
    #                                                      [0, 0, 55301371.20]]  # from Arent

    # envelope properties -> Raymer page 495
    vehicle.envelope.ultimate_load          = 3.75
    vehicle.envelope.limit_load             = 2.5
    vehicle.maximum_mach_operational        = 0.464    # Mission req at 10km -> 500kph

    # basic parameters
    vehicle.reference_area                  = 30  # selected in Optimize.py
    vehicle.systems.control                 = "fully powered"
    vehicle.systems.accessories             = "short-range"

    # ------------------------------------------------------------------
    #  Fuselage - Theoretically OK
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                = 'fuselage'
    fuselage.origin             = [[0, 0, 0]] * Units.meter
    fuselage.number_coach_seats = vehicle.passengers
    fuselage.seat_pitch         = 1 * Units.meter
    fuselage.seats_abreast      = 1  # No seats abreast
    fuselage.fineness.nose      = 6  # Raymer page 157
    fuselage.fineness.tail      = 6
    fuselage.lengths.fore_space = 0  # ???
    fuselage.lengths.aft_space  = 0  # ???
    fuselage.width              = 2.5 * Units.meter  # Guess
    fuselage.heights.maximum    = 3 * Units.meter    # Guess - Aisle height > 1.93 Raymer 9.1 table

    fuselage                      = fuselage_planform(fuselage)  # Returns: Lnose, Ltail, Lcabin, Ltotal,
                                                                    # Swettet, Sfrontproj, Deff
    fuselage.areas.side_projected = ((2 / 3.4) * fuselage.areas.wetted) - fuselage.areas.front_projected  # Raymer p205

    fuselage.heights.at_quarter_length          = 3 * Units.meter  # Guess
    fuselage.heights.at_three_quarters_length   = 3 * Units.meter  # Guess
    fuselage.heights.at_wing_root_quarter_chord = 3 * Units.meter  # Guess

    fuselage.differential_pressure              = 10 ** 5 * Units.pascal  # Maximum differential pressure

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #   Main Wing - Theoretically OK
    # ------------------------------------------------------------------

    wing                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                    = 'main_wing'
    wing.areas.reference        = 30.  # selected in Optimize.py Unknown
    wing.taper                  = 0.45  # Raymer 83
    wing.sweeps.quarter_chord   = 0.0 * Units.deg
    wing.aspect_ratio           = 8  # selected in Optimize.py - Raymer 78
    wing.thickness_to_chord     = 0.14  # In between 0.18 and 0.12 from the airfoils we chose
    wing.dihedral               = 0
    wing.vertical               = False
    wing.symmetric              = True
    wing.origin                 = [[fuselage.lengths.total / 2, 0, fuselage.heights.maximum]] * Units.meter # Almost in the middle, high wing.
    wing.span_efficiency        = 0.8  # Raymer 135

    flap                        = SUAVE.Components.Wings.Control_Surfaces.Flap()
    flap.tag                    = 'flap'
    flap.span_fraction_start    = 0.15  # Initial Guess (Eimai sigouros kapou to leei kai de to vriskw)
    flap.span_fraction_end      = 0.85  # Initial Guess (Eimai sigouros kapou to leei kai de to vriskw)
    flap.deflection             = 0.0 * Units.deg
    flap.chord_fraction         = 0.30  # Raymer page 410
    flap.configuration_type     = 'double_slotted'
    wing.append_control_surface(flap)

    wing = wing_planform(wing)  # Returns: Projected span, Croot, Ctip, Cmac, SweepLE, AffctedArea, WettedArea,
                                # Total span, Aerodynamic center, total length wing

    wing.twists.root            = 0.0 * Units.degrees
    wing.twists.tip             = 0.0 * Units.degrees

    wing.high_lift              = False
    wing.high_mach              = False

    # Probably has to do with drag simulations
    wing.vortex_lift            = False
    # wing.transition_x_upper = 0.2
    # wing.transition_x_lower = 0.25

    wing.dynamic_pressure_ratio = 1.0

    # Method of the repository to calculate Swet with the airfoil and splines.
    # wing.areas.wetted = S_wet_w("supporting/naca64618.dat", wing.taper, wing.areas.reference, wing.spans.projected,
    #                             wing.chords.root,
    #                             100, fuselage.effective_diameter, fuselage.origin[1],
    #                             twin)  # 2.0 * wing.areas.reference

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer // T-Tail configuration - Theoretically OK
    # ------------------------------------------------------------------

    wing                        = SUAVE.Components.Wings.Wing()
    wing.tag                    = 'vertical_stabilizer'
    wing.areas.reference        = 20  # To be optimized.
    wing.taper                  = 0.3  # Raymer table 4.3 p 111
    wing.sweeps.quarter_chord   = 20 * Units.deg  # Raymer 112
    wing.aspect_ratio           = 1.5  # Raymer table 4.3 p 111
    wing.thickness_to_chord     = 0.09  # Typical NACA0009
    wing.span_efficiency        = 0.8  # Guess
    wing.dihedral               = 0
    wing.vertical               = True
    wing.symmetric              = False
    wing.t_tail                 = True

    c_vt                        = 0.08  # Raymer table 6.4 page 160
    l_vt                        = 0.5 * fuselage.lengths.total

    # Kati paizei me auth th kwlomethodo kai kanei return 0.
    wing                        = wing_planform(wing)
    # wing                      = vertical_tail_planform_raymer(wing, vehicle.wings['main_wing'], l_vt, c_vt)
    wing.origin = [[fuselage.lengths.total - wing.chords.root, 0, fuselage.heights.maximum]] * Units.meter

    # wing.areas.reference = (vertical_volume_coefficient * vehicle.wings['main_wing'].spans.projected *
    #                         vehicle.wings['main_wing'].areas.reference) / length_vertical_tail  # Raymer 159

    wing.twists.root            = 0.0 * Units.degrees
    wing.twists.tip             = 0.0 * Units.degrees

    wing.dynamic_pressure_ratio = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer - Theoretically OK
    # ------------------------------------------------------------------

    wing                        = SUAVE.Components.Wings.Wing()
    wing.tag                    = 'horizontal_stabilizer'

    wing.areas.reference        = 10  # To be optimized
    wing.taper                  = 0.5  # Roskam Table 8.14
    wing.sweeps.quarter_chord   = 10 * Units.deg  # Roskam Table 8.14
    wing.aspect_ratio           = 4.0  # Roskam Table 8.14
    wing.thickness_to_chord     = 0.09  # NACA0009
    wing.dihedral               = 0 * Units.degrees
    wing.vertical               = False
    wing.symmetric              = True
    wing.origin                 = [[fuselage.lengths.total - vehicle.wings['vertical_stabilizer'].chords.mean_aerodynamic
                                   , 0, vehicle.wings['vertical_stabilizer'].spans.total / 2]] * Units.meters

    wing.span_efficiency        = 0.9
    l_ht                        = 0.55 * fuselage.lengths.total  # Raymer
    c_ht                        = 0.9  # Raymer table 6.4

    # wing = horizontal_tail_planform_raymer(wing, vehicle.wings['main_wing'], l_ht, c_ht)
    wing                        = wing_planform(wing)  # Without the Raymer corrections

    wing.twists.root            = 0.0 * Units.degrees
    wing.twists.tip             = 0.0 * Units.degrees

    wing.dynamic_pressure_ratio = 1.0  # ?

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Landing gear - Irrelevant for now
    # ------------------------------------------------------------------

    # landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    # landing_gear.tag = "main_landing_gear"
    # landing_gear.main_tire_diameter = 1.35 * Units.m
    # landing_gear.nose_tire_diameter = 0.9 * Units.m
    # landing_gear.main_strut_length = 1.6 * Units.m
    # landing_gear.nose_strut_length = 1.6 * Units.m
    # landing_gear.main_units = 2  # number of main landing gear units
    # landing_gear.nose_units = 1  # number of nose landing gear
    # landing_gear.main_wheels = 4  # number of wheels on the main landing gear
    # landing_gear.main_wheels = 4  # number of wheels on the main landing gear
    # landing_gear.nose_wheels = 2  # number of wheels on the nose landing gear
    # vehicle.landing_gear = landing_gear

    # ------------------------------------------------------------------
    #  Turbofan Network  - To be fixed. We want turboprop.
    # -----------------------------------------------------------------

    vehicle.thrust_total        = 0e3 * Units.N  # defined in Optimize.py / We dont know it for now
    num_engine                  = 2              # Initial Estimate
    bypass                      = 7.5            # Estimate - Historical Data LBR
    # raymer sfc table 3.3

    # design sizing conditions
    altitude                    = 10 * Units.km  # Mission requirement
    mach_number                 = 0.464  # Mission requirement -> 500kph at 10km altitude.
    gt_engine                   = engine_caluclations(altitude, bypass, mach_number, num_engine, vehicle.thrust_total)

    vehicle.append_component(gt_engine)
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # now add weights objects
    vehicle.landing_gear       = SUAVE.Components.Landing_Gear.Landing_Gear()
    vehicle.control_systems    = SUAVE.Components.Physical_Component()
    vehicle.electrical_systems = SUAVE.Components.Physical_Component()
    vehicle.avionics           = SUAVE.Components.Energy.Peripherals.Avionics()
    vehicle.passenger_weights  = SUAVE.Components.Physical_Component()
    vehicle.furnishings        = SUAVE.Components.Physical_Component()
    vehicle.air_conditioner    = SUAVE.Components.Physical_Component()
    vehicle.fuel               = SUAVE.Components.Physical_Component()
    vehicle.apu                = SUAVE.Components.Physical_Component()
    vehicle.hydraulics         = SUAVE.Components.Physical_Component()
    vehicle.optionals          = SUAVE.Components.Physical_Component()

    vehicle.wings['vertical_stabilizer'].rudder = SUAVE.Components.Physical_Component()

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    # config.maximum_lift_coefficient = 1.4

    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings['main_wing'].control_surfaces.flap.angle = 20. * Units.deg

    # config.landing_gear.gear_condition = 'up' # Currently no landing gear on

    configs.append(config)

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].control_surfaces.flap.angle = 30. * Units.deg

    # config.landing_gear.gear_condition = 'down'

    configs.append(config)

    # done!
    return configs

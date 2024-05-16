#ifndef REACSA_CONSTANTS_H_INCLUDED
#define REACSA_CONSTANTS_H_INCLUDED

#include <math.h>
#include <string>

namespace mimpc::systems::reacsa_constants {
// Conversions
    constexpr double S_2_NS = 1e9;             // Seconds to nanoseconds
    constexpr double NS_2_S = 1 / 1e9;         // Nanoseconds to seconds
    constexpr double RAD_2_DEG = 180 / M_PI;   // Radians to degrees
    constexpr double DEG_2_RAD = M_PI / 180;   // Degrees to radians
    constexpr double RPM_2_RADPS = M_PI / 30;  // Rotations per minute to radians per second
    constexpr double RADPS_2_RPM = 30 / M_PI;  // Radians per second to rotations per minute

// Thruster properties
    constexpr size_t NUMBER_THRUSTERS = 8;         // Number of thrusters in REACSA
    constexpr double FORCE_THRUSTER = 10.36;       // Nominal thruster force [N]
    constexpr double TIME_THRUSTER_MIN_OFF = 0.2;  // Minimum off time between firings [s]
    constexpr double TIME_THRUSTER_MIN_ON = 0.1;   // Minimum on time the thruster needs to produce the nominal force [s]
    constexpr double TIME_THRUSTER_MAX_ON = 0.3;   // Maximum on time the thruster can sustain the nominal force [s]
    constexpr double LENGTH_THRUSTER = 0.04;       // Length of a thruster [m]
    constexpr double RADIUS_THRUSTER = 0.02;       // Radius of a thruster [m]
    constexpr double MASS_THRUSTER = 0.01;         // Mass of a thruster [kg]

// Reaction wheel properties
    constexpr double TORQUE_CONSTANT_REACTION_WHEEL = 0.12;  // Reaction wheel torque constant [Nm/A]
    constexpr double TORQUE_REACTION_WHEEL_MAX = 1.44;       // Maximal reaction wheel torque [Nm]
    constexpr double VELOCITY_REACTION_WHEEL_MAX = 600.0;    // Maximum spin velocity of reaction wheel (absolute) [rpm]
    constexpr double VELOCITY_REACTION_WHEEL_MIN = 100.0;    // Minimum spin velocity of reaction wheel (absolute) [rpm]
    constexpr double HEIGHT_REACTION_WHEEL = 0.05;           // Height of the reactionwheel (spinning plate) [m]
    constexpr double RADIUS_REACTION_WHEEL = 0.1285;         // Radius of the reactionwheel (spinning plate) [m]
    constexpr double MASS_REACTION_WHEEL = 4.01;             // Mass of the reactionwheel (spinning plate) [kg]
    constexpr double INERTIA_REACTION_WHEEL = 0.047;         // Reaction wheel moment of inertia [kgm2]

// Air bearing properties (https://www.ibspe.com/air-bearings/flat-air-bearings)
    constexpr double HEIGHT_AIR_BEARING = 0.07;  // Height of an air bearing (without ball-joint leg) [m]
    constexpr double RADIUS_AIR_BEARING = 0.10;  // Radius of an air bearing [m]
    constexpr double MASS_AIR_BEARING = 4.8;     // Mass of an air bearing (without ball-joint leg) [kg]

// Recap properties (recap is simplified to a central cylinder with one top and one bottom plate)
    constexpr double HEIGHT_RECAP_TOP_PLATE = 0.004;     // Height of recap top plate [m]
    constexpr double HEIGHT_RECAP_BOTTOM_PLATE = 0.004;  // Height of recap bottom plate [m]
    constexpr double HEIGHT_RECAP_CENTER = 0.096;        // Height of recap center structure (holding the reactionwheel) [m]
    constexpr double HEIGHT_RECAP_COLUMN = 0.192;        // Height of recap column [m]
    constexpr double RADIUS_RECAP_TOP_PLATE = 0.35;      // Radius of recap top plate [m]
    constexpr double RADIUS_RECAP_BOTTOM_PLATE = 0.35;   // Radius of recap bottom plate [m]
    constexpr double RADIUS_RECAP_CENTER = 0.025;        // Radius of recap center structure (holding the reactionwheel) [m]
    constexpr double RADIUS_RECAP_COLUMN = 0.01;         // Radius of recap column [m]
    constexpr double MASS_RECAP_TOP_PLATE = 5.0;         // Mass of recap top plate [kg]
    constexpr double MASS_RECAP_BOTTOM_PLATE = 6.62;     // Mass of recap bottom plate (including removable equipment) [kg]
    constexpr double MASS_RECAP_CENTER = 1.0;            // Mass of recap center structure (including rw cover) [kg]
    constexpr double MASS_RECAP_COLUMN = 0.16667;        // Mass of recap column [kg]
    constexpr double MASS_RECAP_TOTAL = MASS_RECAP_TOP_PLATE + MASS_RECAP_BOTTOM_PLATE + MASS_RECAP_CENTER
                                        + 6 * MASS_RECAP_COLUMN
                                        + MASS_REACTION_WHEEL;  // Mass of recap platform (including reactionwheel) [kg]
    constexpr double INERTIA_RECAP = 0.67;  // Inertia of recap platform (excluding reactionwheel) [kgm2]

// Satsim properties (satsim is simplified to a central cylinder with one top and one bottom plate)
    constexpr double HEIGHT_SATSIM_TOP_PLATE = 0.006;     // Height of satsim top plate [m]
    constexpr double HEIGHT_SATSIM_BOTTOM_PLATE = 0.006;  // Height of satsim bottom plate [m]
    constexpr double HEIGHT_SATSIM_CENTER = 0.189;        // Height of satsim center structure [m]
    constexpr double RADIUS_SATSIM_TOP_PLATE = 0.35;      // Radius of satsim top plate [m]
    constexpr double RADIUS_SATSIM_BOTTOM_PLATE = 0.35;   // Radius of satsim bottom plate [m]
    constexpr double RADIUS_SATSIM_CENTER = 0.30;         // Radius of satsim center structure [m]
    constexpr double MASS_SATSIM_TOP_PLATE = 5.0;         // Mass of satsim top plate [kg]
    constexpr double MASS_SATSIM_BOTTOM_PLATE = 5.0;      // Mass of satsim bottom plate [kg]
    constexpr double MASS_SATSIM_CENTER = 27.55;          // Mass of satsim center structure (without air) [kg]
    constexpr double MASS_SATSIM_TOTAL =
            MASS_SATSIM_TOP_PLATE + MASS_SATSIM_BOTTOM_PLATE + MASS_SATSIM_CENTER
            + 8 * MASS_THRUSTER;                  // Mass of satsim platform (without air, including thrusters) [kg]
    constexpr double INERTIA_SATSIM = 1.416;  // Inertia of satsim platform (without air) [kgm2]

// Acrobat properties (acrobat is simplified to a central cylinder with one top, one bottom plate and legs to the air
// bearings)
    constexpr double HEIGHT_ACROBAT_TOP_PLATE = 0.006;     // Height of acrobat top plate [m]
    constexpr double HEIGHT_ACROBAT_BOTTOM_PLATE = 0.185;  // Height of acrobat bottom plate [m]
    constexpr double HEIGHT_ACROBAT_CENTER = 0.313;        // Height of acrobat center structure [m]
    constexpr double HEIGHT_ACROBAT_LEG = 0.045;           // Height of acrobat air bearing leg [m]
    constexpr double RADIUS_ACROBAT_TOP_PLATE = 0.35;      // Radius of acrobat top plate [m]
    constexpr double RADIUS_ACROBAT_BOTTOM_PLATE = 0.35;   // Radius of acrobat bottom plate [m]
    constexpr double RADIUS_ACROBAT_CENTER = 0.05;         // Radius of acrobat center structure [m]
    constexpr double RADIUS_ACROBAT_LEG = 0.01;            // Radius of acrobat air bearing leg [m]
    constexpr double RADIUS_ACROBAT_AIR_BEARING = 0.25;    // Radius of air bearing positions under acrobat [m]
    constexpr double MASS_ACROBAT_TOP_PLATE = 5.0;         // Mass of acrobat top plate [kg]
    constexpr double MASS_ACROBAT_BOTTOM_PLATE = 90.15;    // Mass of acrobat bottom plate (including electronics) [kg]
    constexpr double MASS_ACROBAT_CENTER = 35.0;           // Mass of acrobat center structure [kg]
    constexpr double MASS_ACROBAT_LEG = 1.0;               // Mass of acrobat air bearing leg [kg]
    constexpr double MASS_ACROBAT_TOTAL = MASS_ACROBAT_TOP_PLATE + MASS_ACROBAT_BOTTOM_PLATE + MASS_ACROBAT_CENTER
                                          + 3 * MASS_ACROBAT_LEG
                                          + 3 *
                                            MASS_AIR_BEARING;  // Mass of acrobat platform (including air bearings) [kg]
    constexpr double INERTIA_ACROBAT = 10.09;  // Inertia of acrobat platform (including air bearings) [kgm2]

// Reacsa system properties
    constexpr double VELOCITY_REACSA_MAX = 1.0;  // Maximum velocity acceptable for the overall system [m/s]
    constexpr double ROTATIONAL_VELOCITY_REACSA_MAX =
            30.0;                                       // Maximum rotational velocity acceptable for the overall system [deg/s]
    constexpr double ROTATION_REACSA_MAX = 1800.0;  // Maximum rotations acceptable for the overall system [deg]
    constexpr double HEIGHT_REACSA = HEIGHT_RECAP_TOP_PLATE + HEIGHT_RECAP_COLUMN + HEIGHT_RECAP_BOTTOM_PLATE
                                     + HEIGHT_SATSIM_TOP_PLATE + HEIGHT_SATSIM_CENTER + HEIGHT_SATSIM_BOTTOM_PLATE
                                     + HEIGHT_ACROBAT_TOP_PLATE + HEIGHT_ACROBAT_CENTER + HEIGHT_ACROBAT_BOTTOM_PLATE
                                     + HEIGHT_ACROBAT_LEG + HEIGHT_AIR_BEARING;  // Height of the overall system [m]
    constexpr double RADIUS_REACSA = 0.35;                                       // Radius of the overall system [m]
    constexpr double MASS_REACSA =
            MASS_RECAP_TOTAL + MASS_SATSIM_TOTAL + MASS_ACROBAT_TOTAL;  // Mass of the overall system [kg]
    constexpr double INERTIA_REACSA = INERTIA_REACTION_WHEEL + INERTIA_RECAP + INERTIA_SATSIM
                                      + INERTIA_ACROBAT;  // Moment of inertia of the overall system [kgm2]

// Flatfloor properties
    constexpr double LENGTH_FLATFLOOR_X = 4.75;  // Length of flatfloor along (short) x-axis [m]
    constexpr double LENGTH_FLATFLOOR_Y = 8.78;  // Length of flatfloor along (long) y-axis [m]

// Topic names
// The pre-appended "/" has been purposely removed from the beginning of the basenames defined below because they will
// get pre-appended with a namespace if one is defined in the launch file node descriptions or simply a "/" if it is not
    const std::string TOPIC_JOINT_STATES = "joint_states";
    const std::string TOPIC_JOY = "joy";
    const std::string TOPIC_ROBOT_STATE = "robot_state";
    const std::string TOPIC_POSE = "pose";
    const std::string TOPIC_PRESSURE_SENSOR_HIGHLINE =
            "/pressure_sensor_highline/rpdo";  // Adding namespace to ros2 canopen does not work
    const std::string TOPIC_PRESSURE_SENSOR_MIDLINE =
            "/pressure_sensor_midline/rpdo";  // Adding namespace to ros2 canopen does not work
    const std::string TOPIC_SOLENOID_VALVE_STATE = "solenoid_valve_state";
    const std::string TOPIC_PLANNED_TRAJECTORY = "planned_trajectory";
    const std::string TOPIC_MARKER_FLATFLOOR = "marker/flat_floor";
    const std::string TOPIC_MARKER_TELEMETRY = "marker/telemetry";
    const std::string TOPIC_MARKER_PRESSURE_BAR = "marker/pressure_bar";
    const std::string TOPIC_WRENCH_REACTION_WHEEL = "wrench/reaction_wheel";
    const std::string TOPIC_WRENCH_THRUSTERS = "wrench/thrusters";
    const std::string TOPIC_MARKER_PLANNED_TRAJECTORY = "marker/planned_trajectory";
    const std::string TOPIC_MARKER_TRAJECTORY_WAYPOINTS = "marker/trajectory_waypoints";
    const std::string TOPIC_MARKER_WAYPOINT_POSITION =
            "marker/waypoint_position";  // Remove once waypoint markers are concatenated into a single topic
    const std::string TOPIC_MARKER_WAYPOINT_ORIENTATION =
            "marker/waypoint_orientation";  // Remove once waypoint markers are concatenated into a single topic
    const std::string TOPIC_THRUSTERS_COMMAND = "thrusters_command";

// Action names
    const std::string ACTION_COMPUTE_TRAJECTORY = "compute_trajectory";
    const std::string ACTION_FOLLOW_TRAJECTORY = "follow_trajectory";

// Service names
    const std::string SERVICE_SET_SOLENOID_VALVE_STATE = "set_solenoid_valve_state";
    const std::string SERVICE_SET_REACTION_WHEEL_VELOCITY = "set_reaction_wheel_velocity";
    const std::string SERVICE_START_RECORDING = "start_recording";
    const std::string SERVICE_STOP_RECORDING = "stop_recording";
    const std::string SERVICE_RESET_OBSERVER = "reset_observer";
}
#endif  // REACSA_CONSTANTS_H_INCLUDED

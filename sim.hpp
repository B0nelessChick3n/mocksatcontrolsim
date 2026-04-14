/*
███████╗██╗███╗   ███╗   ██╗  ██╗██████╗ ██████╗ 
██╔════╝██║████╗ ████║   ██║  ██║██╔══██╗██╔══██╗
███████╗██║██╔████╔██║   ███████║██████╔╝██████╔╝
╚════██║██║██║╚██╔╝██║   ██╔══██║██╔═══╝ ██╔═══╝ 
███████║██║██║ ╚═╝ ██║██╗██║  ██║██║     ██║     
╚══════╝╚═╝╚═╝     ╚═╝╚═╝╚═╝  ╚═╝╚═╝     ╚═╝     
*/

#pragma once

static constexpr int NUM_MOTORS = 8;

/// Full planar state of the rigid body
struct State {
    float x;       ///< World X position (metres)
    float y;       ///< World Y position (metres)
    float vx;      ///< World X velocity (m/s)
    float vy;      ///< World Y velocity (m/s)
    float angle;   ///< Heading angle (radians, CCW from +x)
    float omega;   ///< Angular velocity (rad/s, CCW positive)
    float time_s;  ///< Simulation time (seconds)
};

/// Motor command vector — each u[i] is normalised to [-1, 1]
struct Control {
    float u[NUM_MOTORS] = {};
};

/// PID gains and desired setpoint — loaded from config.ini at runtime
struct Config {
    // Desired pose
    float goal_x     = 0.f;
    float goal_y     = 0.f;
    float goal_theta = 0.f;

    // Translational PID (applied in body frame to x/y errors)
    float kp_xy  = 0.50f;
    float ki_xy  = 0.02f;
    float kd_xy  = 0.25f;

    // Rotational PID
    float kp_th  = 0.80f;
    float ki_th  = 0.01f;
    float kd_th  = 0.30f;

    // Anti-windup clamp for integrators
    float iclamp_xy = 2.0f;
    float iclamp_th = 1.0f;

    // Vehicle physical properties (applied on simulation reset via R or first launch)
    // Shapes still define collision geometry; mass/inertia are overridden directly.
    float mass_kg = 3.14f;   // total body mass (kg)
    float inertia = 0.50f;   // rotational inertia about CoM (kg·m²)
};

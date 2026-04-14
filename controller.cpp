/*
 ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗     ███████╗██████╗     ██████╗██████╗ ██████╗ 
██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║     ██╔════╝██╔══██╗   ██╔════╝██╔══██╗██╔══██╗
██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║     █████╗  ██████╔╝   ██║     ██████╔╝██████╔╝
██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ██║     ██╔══╝  ██╔══██╗   ██║     ██╔═══╝ ██╔═══╝ 
╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗███████╗███████╗██║  ██║██╗╚██████╗██║     ██║     
 ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝ ╚═════╝╚═╝     ╚═╝     
*/
#include "controller.hpp"
#include <cmath>
#include <algorithm>

/// Wrap angle to [-pi, pi]
static float wrapAngle(float a) {
    while (a >  3.14159265f) a -= 2.f * 3.14159265f;
    while (a < -3.14159265f) a += 2.f * 3.14159265f;
    return a;
}

/// PID integrator state
static float ix = 0.f, iy = 0.f, ith = 0.f;
static bool  first_step = true;

void reset_controller() {
    ix = iy = ith = 0.f;
    first_step = true;
}

void compute_control(const State& s, const Config& cfg, Control& ctrl) {
    for (int i = 0; i < NUM_MOTORS; ++i) ctrl.u[i] = 0.f;

    static constexpr float DT = 0.005f;

    float c  = std::cos(s.angle);
    float si = std::sin(s.angle);

    // ── Position error: world frame → body frame ─────────────────────────────
    float world_ex = cfg.goal_x - s.x;
    float world_ey = cfg.goal_y - s.y;
    float ex =  c * world_ex + si * world_ey;   // body +x = forward
    float ey = -si * world_ex +  c * world_ey;  // body +y = left

    // ── Heading error ────────────────────────────────────────────────────────
    float eth = wrapAngle(cfg.goal_theta - s.angle);

    // ── Integrators ──────────────────────────────────────────────────────────
    if (!first_step) {
        ix  = std::clamp(ix  + ex  * DT, -cfg.iclamp_xy, cfg.iclamp_xy);
        iy  = std::clamp(iy  + ey  * DT, -cfg.iclamp_xy, cfg.iclamp_xy);
        ith = std::clamp(ith + eth * DT, -cfg.iclamp_th, cfg.iclamp_th);
    }
    first_step = false;

    // ── Derivative: body-frame velocity (no frame-rotation spike) ────────────
    float vx_body =  c * s.vx + si * s.vy;
    float vy_body = -si * s.vx +  c * s.vy;
    float dex  = -vx_body;
    float dey  = -vy_body;
    float deth = -s.omega;

    // ── PID outputs ───────────────────────────────────────────────────────────
    float ux  = cfg.kp_xy * ex  + cfg.ki_xy * ix  + cfg.kd_xy * dex;
    float uy  = cfg.kp_xy * ey  + cfg.ki_xy * iy  + cfg.kd_xy * dey;
    float uth = cfg.kp_th * eth + cfg.ki_th * ith  + cfg.kd_th * deth;

    // ── Motor allocation ──────────────────────────────────────────────────────
    //
    // Layout (body frame):
    //   0  pos=(+0.32, +0.12)  thrust +x    front-left
    //   1  pos=(+0.32, -0.12)  thrust +x    front-right
    //   2  pos=(-0.32, +0.12)  thrust -x    back-left
    //   3  pos=(-0.32, -0.12)  thrust -x    back-right
    //   4  pos=(+0.12, +0.32)  thrust +y    left-front
    //   5  pos=(-0.12, +0.32)  thrust +y    left-back
    //   6  pos=(+0.12, -0.32)  thrust -y    right-front
    //   7  pos=(-0.12, -0.32)  thrust -y    right-back
    //
    // Torque analysis (tau = r × F, z-component = x*Fy - y*Fx):
    //
    //   FORWARD pair (motors 0,1): thrust Fx at y=±0.12
    //     tau_z = -y * Fx
    //     motor 0 (y=+0.12): tau = -0.12 * Fx  → CW when Fx>0
    //     motor 1 (y=-0.12): tau = +0.12 * Fx  → CCW when Fx>0
    //     differential u[0]-u[1] gives net torque:
    //       u[0] = ux - uth  →  motor 0 less thrust when uth>0  →  less CW  → net CCW ✓
    //       u[1] = ux + uth  →  motor 1 more thrust when uth>0  →  more CCW ✓
    //
    //   BACK pair (motors 2,3): thrust -Fx at y=±0.12
    //     tau_z = -y * (-Fx) = +y * Fx
    //     motor 2 (y=+0.12): tau = +0.12 * |u| → CCW when braking (u>0 → thrust -x)
    //     motor 3 (y=-0.12): tau = -0.12 * |u| → CW  when braking
    //     mirror the front differential:
    //       u[2] = u_back + uth
    //       u[3] = u_back - uth
    //
    //   LEFT/RIGHT pairs (motors 4-7): thrust ±y at x=±0.12
    //     tau_z = x * Fy
    //     motors 4,5 at x=+0.12 and x=-0.12 with same Fy → torques cancel → zero net tau
    //     motors 6,7 same situation → zero net tau
    //     ∴ left/right pairs are purely for lateral translation, NOT rotation.

    float u_fwd  = (ux > 0.f) ?  ux : 0.f;
    float u_back = (ux < 0.f) ? -ux : 0.f;   // positive value, applied to -x thrusters

    // Front pair: forward drive + rotation differential
    // tau per motor: -y * Fx  →  motor0 y=+0.12 → CW, motor1 y=-0.12 → CCW
    // To get CCW (uth>0): reduce motor0, increase motor1
    ctrl.u[0] = u_fwd - uth;   // front-left  (y=+0.12, CCW: reduce)
    ctrl.u[1] = u_fwd + uth;   // front-right (y=-0.12, CCW: increase)

    // Back pair mirrors front: thrust is -x, so tau = +y*|u|
    // motor2 y=+0.12 → CCW when active, motor3 y=-0.12 → CW when active
    // To get CCW: increase motor2, reduce motor3
    ctrl.u[2] = u_back + uth;  // back-left  (y=+0.12, CCW: increase)
    ctrl.u[3] = u_back - uth;  // back-right (y=-0.12, CCW: reduce)

    // Left/right pairs: pure lateral, no rotation contribution
    ctrl.u[4] =  uy;   // left-front  (+y thrust)
    ctrl.u[5] =  uy;   // left-back   (+y thrust)
    ctrl.u[6] = -uy;   // right-front (-y thrust, negate for same lateral direction)
    ctrl.u[7] = -uy;   // right-back  (-y thrust)
}

```

██████╗ ███████╗ █████╗ ██████╗ ███╗   ███╗███████╗
██╔══██╗██╔════╝██╔══██╗██╔══██╗████╗ ████║██╔════╝
██████╔╝█████╗  ███████║██║  ██║██╔████╔██║█████╗  
██╔══██╗██╔══╝  ██╔══██║██║  ██║██║╚██╔╝██║██╔══╝  
██║  ██║███████╗██║  ██║██████╔╝██║ ╚═╝ ██║███████╗
╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═════╝ ╚═╝     ╚═╝╚══════╝
```
# rigid_body_sim

Deterministic 2D top-down rigid body simulator using **Box2D 2.4** and **SFML 2.6**.

---

## File layout

```
.
├── CMakeLists.txt   — build system (auto-fetches Box2D / SFML if absent)
├── sim.hpp          — State and Control structs, NUM_MOTORS constant
├── controller.hpp   — compute_control() declaration
├── controller.cpp   — YOUR control logic (modify freely)
└── main.cpp         — world setup, sim loop, rendering
```

---

## Quick build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/rigid_body_sim
```

On macOS with Homebrew SFML/Box2D already installed:
```bash
brew install sfml box2d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
```

---

## Design

### Physics (main.cpp)
| Item | Value |
|---|---|
| Timestep `dt` | 0.005 s (200 Hz) |
| Velocity iterations | 8 |
| Position iterations | 3 |
| Gravity | none (top-down) |
| Linear / angular damping | 0.05 |

The simulation loop is **decoupled from rendering**.  
Physics advances by fixed `dt` steps regardless of display frame rate.  
`-fno-fast-math` keeps floating-point strictly IEEE-754 for reproducibility.

### Body geometry
- **Cylinder** — `b2CircleShape` radius 0.4 m, density 5 kg/m²  
- **Box** — `b2PolygonShape` 0.70 × 0.50 m centred at origin, density 2.5 kg/m²  
  Both fixtures are on the *same* `b2Body` (rigid connection, no joint needed).

### Motor model
Eight motors, two per side:

| Index | Side | Position (body frame) | Thrust direction |
|---|---|---|---|
| 0, 1 | Front | (±0.32, ±0.12) | +x (forward) |
| 2, 3 | Back  | (−0.32, ±0.12) | −x (backward) |
| 4, 5 | Left  | (±0.12, +0.32) | +y (left) |
| 6, 7 | Right | (±0.12, −0.32) | −y (right) |

Force = `clamp(GAIN × KV × u, ±MAX_FORCE)` applied via `ApplyForce` at the
motor's world position.  Net force and torque emerge from the geometry.

### Controller interface (`sim.hpp` / `controller.hpp`)
```cpp
struct State  { float x, y, vx, vy, angle, omega, time_s; };
struct Control { float u[8]; };          // u ∈ [-1, 1]
void compute_control(const State&, Control&);
```
Edit **only** `controller.cpp` to change the control law.

### Rendering
- Cylinder → `sf::CircleShape`  
- Box → `sf::RectangleShape` rotated by body angle  
- Motors → coloured dots (green=front, red=back, cyan=left, yellow=right)  
- Scale: 80 px / m, 900 × 900 window  
- Target: 60 FPS (VSync or frame limiter, does **not** affect physics)

### Logging
State is printed to stdout every 200 steps (~1 s wall-clock sim time):
```
[t=1.000] pos=(-0.123, 0.456) vel=(0.78, -0.12) angle=1.23 omega=0.05
```
Pipe to a file for offline analysis:  `./rigid_body_sim > log.txt`

---

## Extending the controller

```cpp
// controller.cpp
void compute_control(const State& s, Control& c) {
    // Example: go-to-origin P controller
    float kp = 0.5f;
    // Forward error in body frame
    float ex =  std::cos(s.angle) * (0 - s.x) + std::sin(s.angle) * (0 - s.y);
    float ey = -std::sin(s.angle) * (0 - s.x) + std::cos(s.angle) * (0 - s.y);
    c.u[0] = c.u[1] = kp * ex;   // front pair → drive toward origin
    c.u[4] = c.u[5] = kp * ey;   // left  pair → strafe
    c.u[2] = c.u[3] = c.u[6] = c.u[7] = 0.f;
}
```

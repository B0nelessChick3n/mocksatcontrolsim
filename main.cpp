// 
// ███╗   ███╗ ██████╗  ██████╗██╗  ██╗███████╗ █████╗ ████████╗    ███████╗██╗███╗   ███╗    ██╗   ██╗ ██╗    ██████╗ 
// ████╗ ████║██╔═══██╗██╔════╝██║ ██╔╝██╔════╝██╔══██╗╚══██╔══╝    ██╔════╝██║████╗ ████║    ██║   ██║███║   ██╔═████╗
// ██╔████╔██║██║   ██║██║     █████╔╝ ███████╗███████║   ██║       ███████╗██║██╔████╔██║    ██║   ██║╚██║   ██║██╔██║
// ██║╚██╔╝██║██║   ██║██║     ██╔═██╗ ╚════██║██╔══██║   ██║       ╚════██║██║██║╚██╔╝██║    ╚██╗ ██╔╝ ██║   ████╔╝██║
// ██║ ╚═╝ ██║╚██████╔╝╚██████╗██║  ██╗███████║██║  ██║   ██║       ███████║██║██║ ╚═╝ ██║     ╚████╔╝  ██║██╗╚██████╔╝
// ╚═╝     ╚═╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝╚═╝╚═╝     ╚═╝      ╚═══╝   ╚═╝╚═╝ ╚═════╝ 
// 
// By: Ryan Coder 04/13/2026
// 
#include "sim.hpp"
#include "controller.hpp"

#include <box2d/box2d.h>
#include <SFML/Graphics.hpp>

#include <cmath>
#include <cstdio>
#include <chrono>
#include <fstream>
#include <string>
#include <random>
#include <algorithm>
#include <filesystem>

// ── Physical constants ───────────────────────────────────────────────────────
static constexpr float CYLINDER_RADIUS  = 0.4f;
static constexpr float BOX_HALF_W       = 0.25f;
static constexpr float BOX_HALF_H       = 0.25f;
static constexpr float WALL_HALF_EXTENT = 5.0f;   // arena is 10m × 10m
static constexpr float WALL_THICKNESS   = 0.2f;

static const b2Vec2 MOTOR_POSITIONS[NUM_MOTORS] = {
    { CYLINDER_RADIUS * 0.8f,  CYLINDER_RADIUS * 0.3f},
    { CYLINDER_RADIUS * 0.8f, -CYLINDER_RADIUS * 0.3f},
    {-CYLINDER_RADIUS * 0.8f,  CYLINDER_RADIUS * 0.3f},
    {-CYLINDER_RADIUS * 0.8f, -CYLINDER_RADIUS * 0.3f},
    { CYLINDER_RADIUS * 0.3f,  CYLINDER_RADIUS * 0.8f},
    {-CYLINDER_RADIUS * 0.3f,  CYLINDER_RADIUS * 0.8f},
    { CYLINDER_RADIUS * 0.3f, -CYLINDER_RADIUS * 0.8f},
    {-CYLINDER_RADIUS * 0.3f, -CYLINDER_RADIUS * 0.8f},
};
static const b2Vec2 MOTOR_DIRECTIONS[NUM_MOTORS] = {
    { 1.f, 0.f}, { 1.f, 0.f},
    {-1.f, 0.f}, {-1.f, 0.f},
    { 0.f, 1.f}, { 0.f, 1.f},
    { 0.f,-1.f}, { 0.f,-1.f},
};

// ── Motor model ──────────────────────────────────────────────────────────────
static constexpr float MOTOR_KV        = 8.0f;   // was 2400 — that saturated every motor to ±MAX_FORCE always
static constexpr float MOTOR_MAX_FORCE = 8.0f;
static constexpr float MOTOR_GAIN      = 1.0f;

// ── Sim timing ───────────────────────────────────────────────────────────────
static constexpr float SIM_DT    = 0.005f;
static constexpr int   VEL_ITERS = 8;
static constexpr float RENDER_FPS = 60.f;

// ── HUD: fixed pixel height strip at the bottom ───────────────────────────────
static constexpr unsigned HUD_H      = 190u;
static constexpr unsigned INIT_WIN_W = 900u;
static constexpr unsigned INIT_WIN_H = 900u + HUD_H;  // sim area = 900×900 square + HUD strip

// ── Perturbation ─────────────────────────────────────────────────────────────
static constexpr float PERTURB_INTERVAL  = 3.0f;   // seconds between auto micro-perturbs
static constexpr float PERTURB_FORCE_MAG = 2.5f;   // N  (raised from 1.2)
static constexpr float PERTURB_TORQUE    = 0.8f;   // N·m (raised from 0.3)

// Manual kick (key P): stronger one-shot impulse
static constexpr float MANUAL_FORCE_MAG  = 6.0f;
static constexpr float MANUAL_TORQUE     = 1.5f;

// Constant drift: slow rotating bias (simulates wind / calibration offset)
static constexpr float DRIFT_FORCE       = 0.35f;  // N — constant lateral push
static constexpr float DRIFT_ROT_SPEED   = 0.11f;  // rad/s — direction slowly rotates

// ─────────────────────────────────────────────────────────────────────────────
// Layout — recomputed every frame from the live window size
// ─────────────────────────────────────────────────────────────────────────────
struct Layout {
    float simW = 900, simH = 900;  // full available sim area (winW × (winH-HUD_H))
    float sqSide = 900;            // actual square side used for rendering
    float ppm  = 80;               // pixels per metre
    float cx   = 450, cy = 450;    // centre of the square in window coords
    float sqTop = 0;               // y-pixel where the square starts

    void update(unsigned winW, unsigned winH) {
        simW = (float)winW;
        simH = (float)winH - (float)HUD_H;
        sqSide = std::min(simW, simH);
        ppm  = sqSide / (2.f * WALL_HALF_EXTENT) * 0.95f;
        // Centre the square horizontally and vertically within the sim area
        cx   = simW * 0.5f;
        sqTop = (simH - sqSide) * 0.5f;
        cy   = sqTop + sqSide * 0.5f;
    }

    sf::Vector2f toScreen(b2Vec2 v) const {
        return { cx + v.x * ppm, cy - v.y * ppm };
    }
    float px(float metres) const { return metres * ppm; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────────────────────────────────────
static float rotToAngle(b2Rot r) { return std::atan2(r.s, r.c); }

static b2Vec2 rotate2D(b2Vec2 v, float a) {
    float c = std::cos(a), s = std::sin(a);
    return { v.x*c - v.y*s, v.x*s + v.y*c };
}

static float wrapAngle(float a) {
    while (a >  3.14159265f) a -= 2.f * 3.14159265f;
    while (a < -3.14159265f) a += 2.f * 3.14159265f;
    return a;
}

static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Config loader
// ─────────────────────────────────────────────────────────────────────────────
static void loadConfig(const std::string& path, Config& cfg) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "[config] Cannot open %s — using defaults\n", path.c_str());
        return;
    }
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        auto hash = val.find('#');
        if (hash != std::string::npos) val = trim(val.substr(0, hash));
        float v = 0.f;
        try { v = std::stof(val); } catch (...) { continue; }
        if      (key == "goal_x")     cfg.goal_x     = v;
        else if (key == "goal_y")     cfg.goal_y     = v;
        else if (key == "goal_theta") cfg.goal_theta = v;
        else if (key == "kp_xy")      cfg.kp_xy      = v;
        else if (key == "ki_xy")      cfg.ki_xy      = v;
        else if (key == "kd_xy")      cfg.kd_xy      = v;
        else if (key == "kp_th")      cfg.kp_th      = v;
        else if (key == "ki_th")      cfg.ki_th      = v;
        else if (key == "kd_th")      cfg.kd_th      = v;
        else if (key == "iclamp_xy")  cfg.iclamp_xy  = v;
        else if (key == "iclamp_th")  cfg.iclamp_th  = v;
        else if (key == "mass_kg")    cfg.mass_kg    = v;
        else if (key == "inertia")    cfg.inertia    = v;
    }
    std::printf("[config] Loaded %s\n"
                "         goal=(%.3f, %.3f, %.1f deg)\n"
                "         kp_xy=%.4f  ki_xy=%.4f  kd_xy=%.4f\n"
                "         kp_th=%.4f  ki_th=%.4f  kd_th=%.4f\n"
                "         iclamp_xy=%.3f  iclamp_th=%.3f\n"
                "         mass_kg=%.3f  inertia=%.4f\n",
        path.c_str(),
        cfg.goal_x, cfg.goal_y, cfg.goal_theta * 180.f / 3.14159265f,
        cfg.kp_xy, cfg.ki_xy, cfg.kd_xy,
        cfg.kp_th, cfg.ki_th, cfg.kd_th,
        cfg.iclamp_xy, cfg.iclamp_th,
        cfg.mass_kg, cfg.inertia);
}

// ─────────────────────────────────────────────────────────────────────────────
// World builder
// ─────────────────────────────────────────────────────────────────────────────
static b2BodyId buildWorld(b2WorldId worldId, std::mt19937& rng, const Config& cfg) {
    auto makeWall = [&](float cx2, float cy2, float hw, float hh) {
        b2BodyDef bd = b2DefaultBodyDef();
        bd.position  = {cx2, cy2};
        b2BodyId wid = b2CreateBody(worldId, &bd);
        b2Polygon box = b2MakeBox(hw, hh);
        b2ShapeDef sd = b2DefaultShapeDef();
        sd.material.restitution = 0.95f;
        sd.material.friction    = 0.0f;
        b2CreatePolygonShape(wid, &sd, &box);
    };
    float e = WALL_HALF_EXTENT, t = WALL_THICKNESS;
    makeWall( 0,  e+t, e+t, t);
    makeWall( 0, -e-t, e+t, t);
    makeWall( e+t, 0,  t, e+t);
    makeWall(-e-t, 0,  t, e+t);

    b2BodyDef bd = b2DefaultBodyDef();
    bd.type           = b2_dynamicBody;
    bd.position       = {0.f, 0.f};
    bd.linearDamping  = 0.05f;
    bd.angularDamping = 0.05f;
    b2BodyId bodyId   = b2CreateBody(worldId, &bd);

    {
        b2ShapeDef sd = b2DefaultShapeDef();
        sd.density              = 5.0f;
        sd.material.friction    = 0.02f;
        sd.material.restitution = 0.05f;
        b2Circle circle = {{0.f, 0.f}, CYLINDER_RADIUS};
        b2CreateCircleShape(bodyId, &sd, &circle);
    }
    {
        b2ShapeDef sd = b2DefaultShapeDef();
        sd.density              = 2.5f;
        sd.material.friction    = 0.02f;
        sd.material.restitution = 0.05f;
        b2Polygon boxShape = b2MakeBox(BOX_HALF_W, BOX_HALF_H);
        b2CreatePolygonShape(bodyId, &sd, &boxShape);
    }

    std::uniform_real_distribution<float> small(-0.15f, 0.15f);
    b2Body_SetLinearVelocity(bodyId, {small(rng), small(rng)});
    b2Body_SetAngularVelocity(bodyId, small(rng));

    // Override mass properties — shapes define collision geometry only
    b2MassData md{};
    md.mass            = std::max(0.01f, cfg.mass_kg);
    md.rotationalInertia = std::max(0.001f, cfg.inertia);
    md.center          = {0.f, 0.f};
    b2Body_SetMassData(bodyId, md);
    std::printf("[body] mass=%.3f kg  inertia=%.4f kg·m²\n", md.mass, md.rotationalInertia);

    return bodyId;
}

// ─────────────────────────────────────────────────────────────────────────────
// Input field widget
// ─────────────────────────────────────────────────────────────────────────────
enum class InputField { None, X, Y, Theta, ManualMag, AutoMag, FreqHz };

struct Field {
    float x = 0, y = 0, w = 0, h = 0;

    bool contains(float mx, float my) const {
        return mx >= x && mx <= x+w && my >= y && my <= y+h;
    }

    void draw(sf::RenderTarget& rt, sf::Font& font,
              const std::string& text, bool isActive, bool blink) const {
        sf::RectangleShape bg({w, h});
        bg.setPosition({x, y});
        bg.setFillColor(isActive ? sf::Color(20, 42, 72) : sf::Color(26, 30, 46));
        bg.setOutlineThickness(isActive ? 2.f : 1.5f);
        bg.setOutlineColor(isActive ? sf::Color(80, 180, 255) : sf::Color(65, 75, 105));
        rt.draw(bg);

        std::string disp = text + (isActive && blink ? "|" : (isActive ? " " : ""));
        sf::Text t(font, disp, 14);
        t.setPosition({x + 7.f, y + (h - 18.f) * 0.5f});
        t.setFillColor(sf::Color(215, 235, 255));
        rt.draw(t);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Goal marker
// ─────────────────────────────────────────────────────────────────────────────
static void drawGoal(sf::RenderTarget& rt, const Layout& L,
                     float gx, float gy, float gth) {
    sf::Vector2f sp = L.toScreen({gx, gy});
    sf::CircleShape ring(9.f);
    ring.setOrigin({9.f, 9.f});
    ring.setPosition(sp);
    ring.setFillColor(sf::Color::Transparent);
    ring.setOutlineThickness(2.f);
    ring.setOutlineColor(sf::Color(255, 220, 50, 210));
    rt.draw(ring);

    float len = 20.f;
    sf::Vertex arrow[2];
    arrow[0].position = sp;
    arrow[0].color    = sf::Color(255, 220, 50, 210);
    arrow[1].position = {sp.x + len * std::cos(-gth), sp.y + len * std::sin(-gth)};
    arrow[1].color    = sf::Color(255, 220, 50, 210);
    rt.draw(arrow, 2, sf::PrimitiveType::Lines);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> perturbDist(-1.f, 1.f);

    // Resolve config.ini relative to the executable, not the working directory.
    // This means it works whether you run ./rigid_body_sim, ./build/rigid_body_sim,
    // or double-click the binary from a file manager.
    std::filesystem::path exeDir = std::filesystem::path(argv[0]).parent_path();
    std::string CFG_PATH = (exeDir / "config.ini").string();
    // Fall back to CWD if not found next to the exe (e.g. during development)
    if (!std::filesystem::exists(CFG_PATH)) {
        CFG_PATH = "config.ini";
        std::fprintf(stderr, "[config] Not found next to executable, trying CWD: %s\n",
            std::filesystem::absolute(CFG_PATH).string().c_str());
    } else {
        std::printf("[config] Using: %s\n", CFG_PATH.c_str());
    }

    Config cfg;
    loadConfig(CFG_PATH, cfg);

    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity    = {0.f, 0.f};
    b2WorldId worldId = b2CreateWorld(&wd);
    b2BodyId  bodyId  = buildWorld(worldId, rng, cfg);

    sf::RenderWindow window(
        sf::VideoMode({INIT_WIN_W, INIT_WIN_H}),
        "Rigid Body Sim — R=Reset  L=Reload  Tab/Click=Select  Enter=Apply"
    );
    window.setFramerateLimit(0);

    // Font — try common monospace paths across Linux / macOS / Windows
    sf::Font font;
    bool fontLoaded =
        font.openFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf") ||
        font.openFromFile("/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf") ||
        font.openFromFile("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf") ||
        font.openFromFile("/System/Library/Fonts/Menlo.ttc") ||
        font.openFromFile("/System/Library/Fonts/Monaco.ttf") ||
        font.openFromFile("C:/Windows/Fonts/consola.ttf") ||
        font.openFromFile("C:/Windows/Fonts/cour.ttf");

    if (!fontLoaded)
        std::fprintf(stderr, "[font] No font found — HUD text will not render\n");

    // Input state
    InputField activeField = InputField::None;
    auto fmtF = [](float v) {
        char b[32]; std::snprintf(b, sizeof(b), "%.3f", v); return std::string(b);
    };
    std::string bufX  = fmtF(cfg.goal_x);
    std::string bufY  = fmtF(cfg.goal_y);
    std::string bufTh = fmtF(cfg.goal_theta * 180.f / 3.14159265f);
    std::string statusMsg = "Ready — click a field or press Tab, type value, press Enter";

    // Runtime-editable perturbation magnitudes
    float manualForceMag = MANUAL_FORCE_MAG;
    float autoForceMag   = PERTURB_FORCE_MAG;
    float perturbFreqHz  = 1.f / PERTURB_INTERVAL;   // default: 1/3 Hz
    std::string bufManual = fmtF(manualForceMag);
    std::string bufAuto   = fmtF(autoForceMag);
    std::string bufFreq   = fmtF(perturbFreqHz);

    using Clock = std::chrono::steady_clock;
    auto lastRender = Clock::now();
    auto lastTime   = Clock::now();
    double accumulator = 0.0;
    uint64_t stepCount = 0;
    float nextPerturb  = PERTURB_INTERVAL;

    State   state{};
    Control ctrl{};
    Layout  L;

    bool  manualKickPending = false;   // set by key P, consumed in physics loop
    float perturbFlashTimer = 0.f;     // seconds remaining to show the flash indicator
    float driftAngle        = 0.f;     // current direction of the constant drift force

    while (window.isOpen()) {

        // Recompute layout from the live window dimensions
        auto [winW, winH] = window.getSize();
        L.update(winW, winH);

        // ── Derive HUD geometry from current window size ──────────────────────
        // Three equal columns for the three input fields
        float pad    = 10.f;
        float colW   = ((float)winW - pad * 4.f) / 3.f;
        float col0   = pad;
        float col1   = pad + colW + pad;
        float col2   = pad + 2.f * (colW + pad);
        float hudY   = (float)winH - (float)HUD_H;   // top of HUD strip in window coords
        float labelY = hudY + 6.f;
        float fieldY = labelY + 18.f;
        float fieldH = 28.f;

        // Second row: perturbation magnitude + frequency fields (three equal columns)
        float label2Y  = fieldY + fieldH + 8.f;
        float field2Y  = label2Y + 18.f;
        float pertColW = ((float)winW - pad * 4.f) / 3.f;
        float pertCol0 = pad;
        float pertCol1 = pad + pertColW + pad;
        float pertCol2 = pad + 2.f * (pertColW + pad);

        float statusY = field2Y + fieldH + 5.f;
        float hintsY  = statusY + 16.f;
        float pidY    = hintsY  + 15.f;

        Field fX  { col0, fieldY, colW, fieldH };
        Field fY  { col1, fieldY, colW, fieldH };
        Field fTh { col2, fieldY, colW, fieldH };
        Field fManual { pertCol0, field2Y, pertColW, fieldH };
        Field fAuto   { pertCol1, field2Y, pertColW, fieldH };
        Field fFreq   { pertCol2, field2Y, pertColW, fieldH };

        // ── Event handling ────────────────────────────────────────────────────
        while (const std::optional ev = window.pollEvent()) {
            if (ev->is<sf::Event::Closed>()) window.close();

            if (const auto* kp = ev->getIf<sf::Event::KeyPressed>()) {
                auto sc = kp->scancode;

                if (sc == sf::Keyboard::Scancode::Escape) {
                    if (activeField != InputField::None) activeField = InputField::None;
                    else window.close();
                }

                if (sc == sf::Keyboard::Scancode::R && activeField == InputField::None) {
                    b2DestroyWorld(worldId);
                    worldId     = b2CreateWorld(&wd);
                    bodyId      = buildWorld(worldId, rng, cfg);
                    reset_controller();
                    stepCount   = 0;
                    accumulator = 0.0;
                    nextPerturb = PERTURB_INTERVAL;
                    driftAngle  = 0.f;
                    perturbFlashTimer = 0.f;
                    manualKickPending = false;
                    statusMsg   = "Simulation reset";
                    std::printf("[sim] Reset\n");
                }

                if (sc == sf::Keyboard::Scancode::L && activeField == InputField::None) {
                    loadConfig(CFG_PATH, cfg);
                    reset_controller();
                    bufX  = fmtF(cfg.goal_x);
                    bufY  = fmtF(cfg.goal_y);
                    bufTh = fmtF(cfg.goal_theta * 180.f / 3.14159265f);
                    statusMsg = "Reloaded " + CFG_PATH;
                }

                if (sc == sf::Keyboard::Scancode::P && activeField == InputField::None) {
                    manualKickPending = true;
                }

                if (sc == sf::Keyboard::Scancode::Tab) {
                    bool shift = kp->shift;
                    if (shift) {
                        if      (activeField == InputField::None)      activeField = InputField::FreqHz;
                        else if (activeField == InputField::FreqHz)    activeField = InputField::AutoMag;
                        else if (activeField == InputField::AutoMag)   activeField = InputField::ManualMag;
                        else if (activeField == InputField::ManualMag) activeField = InputField::Theta;
                        else if (activeField == InputField::Theta)     activeField = InputField::Y;
                        else if (activeField == InputField::Y)         activeField = InputField::X;
                        else                                            activeField = InputField::None;
                    } else {
                        if      (activeField == InputField::None)      activeField = InputField::X;
                        else if (activeField == InputField::X)         activeField = InputField::Y;
                        else if (activeField == InputField::Y)         activeField = InputField::Theta;
                        else if (activeField == InputField::Theta)     activeField = InputField::ManualMag;
                        else if (activeField == InputField::ManualMag) activeField = InputField::AutoMag;
                        else if (activeField == InputField::AutoMag)   activeField = InputField::FreqHz;
                        else                                            activeField = InputField::None;
                    }
                }

                if (sc == sf::Keyboard::Scancode::Backspace) {
                    auto* buf =
                        activeField == InputField::X         ? &bufX      :
                        activeField == InputField::Y         ? &bufY      :
                        activeField == InputField::Theta     ? &bufTh     :
                        activeField == InputField::ManualMag ? &bufManual :
                        activeField == InputField::AutoMag   ? &bufAuto   :
                        activeField == InputField::FreqHz    ? &bufFreq   : nullptr;
                    if (buf && !buf->empty()) buf->pop_back();
                }

                if (sc == sf::Keyboard::Scancode::Enter) {
                    try {
                        cfg.goal_x     = std::stof(bufX);
                        cfg.goal_y     = std::stof(bufY);
                        cfg.goal_theta = std::stof(bufTh) * 3.14159265f / 180.f;
                        float newManual = std::max(0.f, std::stof(bufManual));
                        float newAuto   = std::max(0.f, std::stof(bufAuto));
                        float newFreq   = std::max(0.f, std::stof(bufFreq));
                        manualForceMag  = newManual;
                        autoForceMag    = newAuto;
                        perturbFreqHz   = newFreq;
                        bufManual = fmtF(manualForceMag);
                        bufAuto   = fmtF(autoForceMag);
                        bufFreq   = fmtF(perturbFreqHz);
                        // reschedule next perturb based on new frequency
                        if (perturbFreqHz > 0.f)
                            nextPerturb = state.time_s + 1.f / perturbFreqHz;
                        reset_controller();
                        statusMsg = "Applied — goal=(" + bufX + "," + bufY + ") kick=" +
                                    bufManual + "N auto=" + bufAuto + "N";
                        std::printf("[goal] x=%.3f y=%.3f theta=%.3f deg  "
                                    "manualMag=%.2f autoMag=%.2f\n",
                            cfg.goal_x, cfg.goal_y,
                            cfg.goal_theta * 180.f / 3.14159265f,
                            manualForceMag, autoForceMag);
                    } catch (...) {
                        statusMsg = "Parse error — enter valid numbers";
                    }
                    activeField = InputField::None;
                }
            }

            // Printable text into the active field
            if (const auto* te = ev->getIf<sf::Event::TextEntered>()) {
                uint32_t ch = te->unicode;
                auto* buf =
                    activeField == InputField::X         ? &bufX      :
                    activeField == InputField::Y         ? &bufY      :
                    activeField == InputField::Theta     ? &bufTh     :
                    activeField == InputField::ManualMag ? &bufManual :
                    activeField == InputField::AutoMag   ? &bufAuto   :
                    activeField == InputField::FreqHz    ? &bufFreq   : nullptr;
                if (buf) {
                    if ((ch >= '0' && ch <= '9') || ch == '.') {
                        buf->push_back((char)ch);
                    } else if (ch == '-') {
                        // Toggle leading minus: add if empty, flip if already has one
                        if (buf->empty()) {
                            buf->push_back('-');
                        } else if ((*buf)[0] == '-') {
                            buf->erase(buf->begin());
                        } else {
                            buf->insert(buf->begin(), '-');
                        }
                    }
                }
            }

            // Mouse click — hit-test against the field rects
            if (const auto* mc = ev->getIf<sf::Event::MouseButtonPressed>()) {
                float mx = (float)mc->position.x;
                float my = (float)mc->position.y;
                InputField clicked = InputField::None;
                if      (fX.contains(mx, my))      clicked = InputField::X;
                else if (fY.contains(mx, my))      clicked = InputField::Y;
                else if (fTh.contains(mx, my))     clicked = InputField::Theta;
                else if (fManual.contains(mx, my)) clicked = InputField::ManualMag;
                else if (fAuto.contains(mx, my))   clicked = InputField::AutoMag;
                else if (fFreq.contains(mx, my))   clicked = InputField::FreqHz;

                if (clicked != InputField::None) {
                    // Clear buffer when clicking a new field so typing replaces the value
                    if (clicked != activeField) {
                        if (clicked == InputField::X)         bufX.clear();
                        if (clicked == InputField::Y)         bufY.clear();
                        if (clicked == InputField::Theta)     bufTh.clear();
                        if (clicked == InputField::ManualMag) bufManual.clear();
                        if (clicked == InputField::AutoMag)   bufAuto.clear();
                        if (clicked == InputField::FreqHz)    bufFreq.clear();
                    }
                    activeField = clicked;
                } else if (my > (float)winH - (float)HUD_H) {
                    activeField = InputField::None;
                }
                // clicks inside the sim view leave the active field unchanged
            }
        }

        // ── Physics step ──────────────────────────────────────────────────────
        auto now    = Clock::now();
        double elapsed = std::chrono::duration<double>(now - lastTime).count();
        lastTime    = now;
        if (elapsed > 0.1) elapsed = 0.1;
        accumulator += elapsed;

        while (accumulator >= SIM_DT) {
            accumulator -= SIM_DT;

            b2Vec2 pos = b2Body_GetPosition(bodyId);
            b2Vec2 vel = b2Body_GetLinearVelocity(bodyId);
            float  ang = rotToAngle(b2Body_GetRotation(bodyId));

            state.x      = pos.x;
            state.y      = pos.y;
            state.vx     = vel.x;
            state.vy     = vel.y;
            state.angle  = ang;
            state.omega  = b2Body_GetAngularVelocity(bodyId);
            state.time_s = (float)stepCount * SIM_DT;

            // ── Constant drift force (slowly-rotating bias) ───────────────────
            driftAngle += DRIFT_ROT_SPEED * SIM_DT;
            b2Vec2 drift = { DRIFT_FORCE * std::cos(driftAngle),
                             DRIFT_FORCE * std::sin(driftAngle) };
            b2Body_ApplyForceToCenter(bodyId, drift, true);

            // ── Periodic auto micro-perturbation ─────────────────────────────
            float perturbInterval = (perturbFreqHz > 0.f) ? (1.f / perturbFreqHz) : 1e9f;
            if (state.time_s >= nextPerturb) {
                nextPerturb += perturbInterval;
                b2Vec2 fp = { perturbDist(rng) * autoForceMag,
                              perturbDist(rng) * autoForceMag };
                b2Body_ApplyLinearImpulseToCenter(bodyId, fp, true);
                b2Body_ApplyAngularImpulse(bodyId, perturbDist(rng) * PERTURB_TORQUE, true);
                perturbFlashTimer = 0.4f;
                std::printf("[perturb] Auto kick (%.2f N·s)\n", autoForceMag);
            }

            // ── Manual kick (key P) ───────────────────────────────────────────
            if (manualKickPending) {
                manualKickPending = false;
                b2Vec2 fp = { perturbDist(rng) * manualForceMag,
                              perturbDist(rng) * manualForceMag };
                b2Body_ApplyLinearImpulseToCenter(bodyId, fp, true);
                b2Body_ApplyAngularImpulse(bodyId, perturbDist(rng) * MANUAL_TORQUE, true);
                perturbFlashTimer = 0.6f;
                std::printf("[perturb] Manual kick applied (%.2f N)\n", manualForceMag);
            }

            perturbFlashTimer = std::max(0.f, perturbFlashTimer - SIM_DT);

            compute_control(state, cfg, ctrl);

            for (int i = 0; i < NUM_MOTORS; ++i) {
                float u = std::clamp(ctrl.u[i], -1.f, 1.f);
                float fm = std::clamp(MOTOR_GAIN * MOTOR_KV * u,
                                      -MOTOR_MAX_FORCE, MOTOR_MAX_FORCE);
                b2Vec2 dir = rotate2D(MOTOR_DIRECTIONS[i], ang);
                b2Vec2 rpt = rotate2D(MOTOR_POSITIONS[i],  ang);
                b2Vec2 pt  = {pos.x + rpt.x, pos.y + rpt.y};
                b2Body_ApplyForce(bodyId, {dir.x*fm, dir.y*fm}, pt, true);
            }

            b2World_Step(worldId, SIM_DT, VEL_ITERS);
            ++stepCount;

            if (stepCount % 200 == 0)
                std::printf("[t=%.2f] pos=(%.3f,%.3f) ang=%.1f deg  "
                            "goal=(%.2f,%.2f,%.1f deg)\n",
                    state.time_s, state.x, state.y,
                    state.angle * 180.f / 3.14159265f,
                    cfg.goal_x, cfg.goal_y,
                    cfg.goal_theta * 180.f / 3.14159265f);
        }

        // ── Render ────────────────────────────────────────────────────────────
        {
            auto nowR    = Clock::now();
            float sinceR = std::chrono::duration<float>(nowR - lastRender).count();
            if (sinceR < 1.f / RENDER_FPS) continue;
            lastRender = nowR;
        }

        b2Vec2 pos   = b2Body_GetPosition(bodyId);
        float  angle = rotToAngle(b2Body_GetRotation(bodyId));
        float  sfDeg = -angle * 180.f / 3.14159265f;
        sf::Vector2f sp = L.toScreen(pos);

        // Reset to pixel-perfect view so SFML never stretches on resize
        window.setView(sf::View(sf::FloatRect({0.f, 0.f},
            {(float)winW, (float)winH})));

        window.clear(sf::Color(18, 18, 24));

        // Arena
        {
            float side = L.px(WALL_HALF_EXTENT * 2.f);
            sf::RectangleShape arena({side, side});
            arena.setOrigin({side * 0.5f, side * 0.5f});
            arena.setPosition(L.toScreen({0.f, 0.f}));
            arena.setFillColor(sf::Color(28, 30, 38));
            arena.setOutlineThickness(3.f);
            arena.setOutlineColor(sf::Color(150, 155, 175));
            window.draw(arena);
        }

        drawGoal(window, L, cfg.goal_x, cfg.goal_y, cfg.goal_theta);

        // Circle body
        {
            float r = L.px(CYLINDER_RADIUS);
            sf::CircleShape cyl(r);
            cyl.setOrigin({r, r});
            cyl.setPosition(sp);
            cyl.setFillColor(sf::Color(70, 130, 180, 200));
            cyl.setOutlineThickness(2.f);
            cyl.setOutlineColor(sf::Color::White);
            window.draw(cyl);
        }

        // Box fixture
        {
            sf::RectangleShape box({L.px(BOX_HALF_W*2), L.px(BOX_HALF_H*2)});
            box.setOrigin({L.px(BOX_HALF_W), L.px(BOX_HALF_H)});
            box.setPosition(sp);
            box.setRotation(sf::degrees(sfDeg));
            box.setFillColor(sf::Color(200, 100, 50, 180));
            box.setOutlineThickness(1.5f);
            box.setOutlineColor(sf::Color(255, 200, 100));
            window.draw(box);
        }

        // Motor dots
        {
            float mr = std::max(3.f, L.px(0.055f));
            sf::CircleShape mark(mr);
            mark.setOrigin({mr, mr});
            for (int i = 0; i < NUM_MOTORS; ++i) {
                b2Vec2 mp = rotate2D(MOTOR_POSITIONS[i], angle);
                mark.setPosition(L.toScreen({pos.x + mp.x, pos.y + mp.y}));
                mark.setFillColor(
                    i < 2 ? sf::Color(100, 255, 100) :
                    i < 4 ? sf::Color(255,  80,  80) :
                    i < 6 ? sf::Color( 80, 220, 255) :
                             sf::Color(255, 220,  50));
                window.draw(mark);
            }
        }

        // ── HUD strip ─────────────────────────────────────────────────────────
        {
            float hudY = (float)winH - (float)HUD_H;
            sf::RectangleShape hudBg({(float)winW, (float)HUD_H});
            hudBg.setPosition({0.f, hudY});
            hudBg.setFillColor(sf::Color(10, 11, 18));
            window.draw(hudBg);

            sf::RectangleShape sep({(float)winW, 1.5f});
            sep.setPosition({0.f, hudY});
            sep.setFillColor(sf::Color(55, 60, 85));
            window.draw(sep);
        }

        if (fontLoaded) {
            bool blink = (std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now().time_since_epoch()).count() / 500) % 2 == 0;

            // Column labels
            auto lbl = [&](const char* s, float x, float y) {
                sf::Text t(font, s, 12);
                t.setPosition({x, y});
                t.setFillColor(sf::Color(120, 160, 210));
                window.draw(t);
            };
            lbl("Goal X  (m)", col0, labelY);
            lbl("Goal Y  (m)", col1, labelY);
            lbl("Goal theta (deg)", col2, labelY);

            fX.draw (window, font, bufX,  activeField == InputField::X,     blink);
            fY.draw (window, font, bufY,  activeField == InputField::Y,     blink);
            fTh.draw(window, font, bufTh, activeField == InputField::Theta, blink);

            // Second row: perturbation magnitude fields
            lbl("Manual kick (N*s) [P]", pertCol0, label2Y);
            lbl("Auto perturb (N*s)", pertCol1, label2Y);
            lbl("Perturb freq (Hz)", pertCol2, label2Y);
            fManual.draw(window, font, bufManual, activeField == InputField::ManualMag, blink);
            fAuto.draw  (window, font, bufAuto,   activeField == InputField::AutoMag,   blink);
            fFreq.draw  (window, font, bufFreq,   activeField == InputField::FreqHz,    blink);

            // Status
            {
                sf::Text t(font, statusMsg, 12);
                t.setPosition({col0, statusY});
                t.setFillColor(sf::Color(90, 210, 130));
                window.draw(t);
            }

            // Perturbation flash indicator
            if (perturbFlashTimer > 0.f) {
                sf::Text t(font, "** PERTURB **", 13);
                t.setPosition({(float)winW - 150.f, statusY});
                uint8_t alpha = (uint8_t)(255.f * std::min(1.f, perturbFlashTimer / 0.3f));
                t.setFillColor(sf::Color(255, 100, 60, alpha));
                window.draw(t);
            }

            // Drift direction arrow (small compass in corner)
            {
                float ax = (float)winW - 28.f;
                float ay = statusY + 8.f;
                float len = 14.f;
                sf::Vertex arr[2];
                arr[0].position = {ax, ay};
                arr[0].color    = sf::Color(120, 200, 255, 160);
                arr[1].position = {ax + len * std::cos(driftAngle),
                                   ay - len * std::sin(driftAngle)};
                arr[1].color    = sf::Color(120, 200, 255, 160);
                window.draw(arr, 2, sf::PrimitiveType::Lines);

                sf::CircleShape dot(2.f);
                dot.setOrigin({2.f, 2.f});
                dot.setPosition({ax, ay});
                dot.setFillColor(sf::Color(120, 200, 255, 160));
                window.draw(dot);
            }

            // Key hints
            {
                sf::Text t(font,
                    "[R] Reset   [L] Reload config.ini   [Tab/Shift+Tab] Cycle fields   "
                    "[Enter] Apply   [P] Manual kick   [Esc] Deselect/Quit",
                    11);
                t.setPosition({col0, hintsY});
                t.setFillColor(sf::Color(80, 88, 115));
                window.draw(t);
            }

            // State + PID summary
            {
                b2Vec2 v2 = b2Body_GetLinearVelocity(bodyId);
                float ex = cfg.goal_x - pos.x, ey2 = cfg.goal_y - pos.y;
                float dist = std::sqrt(ex*ex + ey2*ey2);
                char buf[400];
                std::snprintf(buf, sizeof(buf),
                    "pos(%.2f,%.2f) ang=%.1f deg  "
                    "err dist=%.3fm dang=%.1f deg  "
                    "kpXY=%.2f kdXY=%.2f  kpTh=%.2f kdTh=%.2f  "
                    "mass=%.2fkg  I=%.3f kg·m²  t=%.1fs",
                    pos.x, pos.y, angle * 180.f / 3.14159265f,
                    dist,
                    wrapAngle(cfg.goal_theta - angle) * 180.f / 3.14159265f,
                    cfg.kp_xy, cfg.kd_xy, cfg.kp_th, cfg.kd_th,
                    cfg.mass_kg, cfg.inertia,
                    state.time_s);
                sf::Text t(font, buf, 11);
                t.setPosition({col0, pidY});
                t.setFillColor(sf::Color(75, 88, 118));
                window.draw(t);
            }
        }

        window.display();
    }

    b2DestroyWorld(worldId);
    return 0;
}

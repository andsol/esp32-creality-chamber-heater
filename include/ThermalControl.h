#pragma once
#include <cmath>
#include <cstdint>

namespace thermal {
inline double clamp(double x, double low, double high) {
    return x < low ? low : (x > high ? high : x);
}
struct Config {
    // Zero means not commissioned; do not guess the assembly's thermal limits.
    double soft = 0, hard = 0, minFan = 0;
    double maxFan = 100, maxHeat = 86.0;
    double kp = 2.0, ki = 0.02; // %/C and %/(C*s), starting values, require tuning.
    double verifySeconds = 600; // Continuous >50% heat, >=2 C below target.
    bool valid() const {
        return std::isfinite(soft) && std::isfinite(hard) &&
            std::isfinite(minFan) && std::isfinite(maxFan) &&
            std::isfinite(maxHeat) && std::isfinite(kp) && std::isfinite(ki) &&
            std::isfinite(verifySeconds) && soft >= 30 && hard >= soft + 5 && hard <= 135 &&
            minFan >= 10 && minFan <= maxFan && maxFan <= 100 &&
            maxHeat > 0 && maxHeat <= 86 && kp > 0 && kp <= 100 &&
            ki >= 0 && ki <= 10 && verifySeconds >= 60 && verifySeconds <= 3600;
    }
};
struct Input {
    double bottom = NAN, top = NAN, radiator = NAN;
    bool bottomValid = false, topValid = false, radiatorValid = false;
    bool useBottom = true;
    double target = 0;
};
enum class State { Idle, Unconfigured, Spinup, Heating, Cooldown, Fault };
class Controller {
public:
    Config config;
    State state = State::Idle;
    double heater = 0, fan = 0;
    const char* fault = "";
    static constexpr double chamberLimit = 70;
    const char* stateName() const {
        switch (state) {
            case State::Idle: return "Idle";
            case State::Unconfigured: return "Setup required";
            case State::Spinup: return "Fan spin-up";
            case State::Heating: return "Heating";
            case State::Cooldown: return "Cooldown";
            case State::Fault: return "Fault";
        }
        return "Unknown";
    }
    bool localValid(const Input& x) const {
        return x.bottomValid && x.radiatorValid && std::isfinite(x.bottom) &&
            std::isfinite(x.radiator);
    }
    bool sensorsValid(const Input& x) const {
        return localValid(x) && (x.useBottom || (x.topValid && std::isfinite(x.top)));
    }
    bool cool(const Input& x) const {
        return localValid(x) && x.bottom < chamberLimit - 2 &&
            (!x.topValid || x.top < chamberLimit - 2) &&
            x.radiator <= x.bottom + 5 &&
            (!config.valid() || x.radiator < config.soft - 2);
    }
    void trip(const char* reason) {
        if (state != State::Fault) fault = reason;
        state = State::Fault;
        heater = 0;
        integral = 0;
        coolSeconds = 0;
        fan = 100;
    }
    bool reset(const Input& x) {
        if (x.target != 0 || !config.valid() || !sensorsValid(x) || !cool(x) ||
            (state == State::Fault && coolSeconds < 30)) return false;
        fault = "";
        state = State::Idle;
        heater = fan = integral = 0;
        coolSeconds = verifyElapsed = 0;
        return true;
    }
    void tick(const Input& x, uint32_t now) {
        const uint32_t elapsed = now - lastTick;
        double dt = started ? elapsed / 1000.0 : 0;
        lastTick = now;
        started = true;
        if (elapsed > 2500 && (state == State::Heating || state == State::Spinup))
            trip("Control loop delayed");
        dt = clamp(dt, 0, 2.5);
        if (!std::isfinite(x.target) || x.target < 0 || x.target > 65)
            trip("Invalid target");
        if ((x.bottomValid && x.bottom >= chamberLimit) ||
            (x.topValid && x.top >= chamberLimit)) trip("Chamber overtemperature");
        if (config.valid() && x.radiatorValid && x.radiator >= config.hard)
            trip("Radiator overtemperature");
        if ((state == State::Heating || state == State::Spinup || x.target > 0) &&
            config.valid() && !sensorsValid(x)) trip("Temperature sensor missing or stale");
        if (state == State::Fault) {
            heater = 0;
            coolSeconds = cool(x) ? coolSeconds + dt : 0;
            fan = coolSeconds >= 30 ? 0 : 100;
            return;
        }
        if (!config.valid()) {
            if (state != State::Unconfigured) coolSeconds = 0;
            state = State::Unconfigured;
            heater = integral = 0;
            // Require sustained cooling confirmation; noisy readings must not chatter the fan.
            coolSeconds = cool(x) ? clamp(coolSeconds + dt, 0, 30) : 0;
            fan = coolSeconds >= 30 ? 0 : 100;
            return;
        }
        if (x.target == 0) {
            heater = integral = 0;
            verifyElapsed = 0;
            if (state == State::Heating || state == State::Spinup ||
                (!cool(x) && state != State::Cooldown)) {
                state = State::Cooldown;
                spinStarted = now;
                coolSeconds = 0;
            }
            if (state == State::Cooldown) {
                coolSeconds = cool(x) ? coolSeconds + dt : 0;
                if (coolSeconds >= 30) { state = State::Idle; fan = 0; }
                else fan = static_cast<uint32_t>(now - spinStarted) < 2000 ||
                    !localValid(x) || x.radiator >= config.soft ? 100 : config.minFan;
            } else { state = State::Idle; fan = 0; }
            return;
        }
        if (state != State::Heating && state != State::Spinup) {
            state = State::Spinup;
            spinStarted = now;
            integral = verifyElapsed = 0;
            previousTarget = x.target;
            heater = 0;
            fan = 100;
        }
        if (state == State::Spinup) {
            heater = 0;
            fan = 100;
            if (static_cast<uint32_t>(now - spinStarted) < 2000) return;
            state = State::Heating;
        }
        const double temperature = x.useBottom ? x.bottom : x.top;
        const double error = x.target - temperature;
        if (x.target != previousTarget) { integral = verifyElapsed = 0; previousTarget = x.target; }
        double cap = config.maxHeat;
        if (x.radiator > config.soft)
            cap *= clamp((config.hard - x.radiator) / (config.hard - config.soft), 0, 1);
        if (error < -0.5) cap = 0;
        const double candidate = clamp(integral + config.ki * error * dt, 0, config.maxHeat);
        // Conditional integration: freeze while upper saturation blocks heating;
        // allow negative error to unwind stored demand.
        if (config.kp * error + candidate <= cap || error < 0) integral = candidate;
        heater = clamp(config.kp * error + integral, 0, cap);
        double wanted = config.minFan + (config.maxFan - config.minFan) * heater / config.maxHeat;
        // Begin extra cooling 10 C below the soft limit; reach 100% at soft.
        const double thermalFan = 100 * clamp((x.radiator - (config.soft - 10)) / 10, 0, 1);
        if (thermalFan > wanted) wanted = thermalFan;
        // Immediate increases, slow decreases (2 percentage points/s), 1% deadband.
        if (wanted > fan) fan = wanted;
        else if (fan - wanted > 1) fan = fan - wanted < 2 * dt ? wanted : fan - 2 * dt;
        fan = clamp(fan, config.minFan, 100);
        if (heater >= 50 && error >= 2) {
            if (verifyElapsed == 0) verifyStart = temperature;
            verifyElapsed += dt;
            if (temperature >= verifyStart + 1) { verifyElapsed = 0; }
            else if (verifyElapsed >= config.verifySeconds) trip("No chamber temperature rise");
        } else verifyElapsed = 0;
    }
private:
    bool started = false;
    uint32_t lastTick = 0, spinStarted = 0;
    double integral = 0, coolSeconds = 0, previousTarget = 0;
    double verifyElapsed = 0, verifyStart = 0;
};
} // namespace thermal

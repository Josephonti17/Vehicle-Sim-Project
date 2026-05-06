// Driver.cpp
#include "Driver.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "Vehicle.h"
#include <cmath>

// ---------- AggressiveDriver ----------

AggressiveDriver::AggressiveDriver(double shiftUpRpm, double shiftDownRpm)
    : shiftUpRpm_(shiftUpRpm), shiftDownRpm_(shiftDownRpm) {}

ShiftAction AggressiveDriver::shiftDecision(const DriverState& state,
                                            const Vehicle& vehicle) const {
    const auto& trans = vehicle.transmission();
    if (state.engineRpm >= shiftUpRpm_ &&
        state.currentGear + 1 < trans.numGears()) {
        return ShiftAction::Up;
    }
    if (state.engineRpm < shiftDownRpm_ && state.currentGear > 0) {
        return ShiftAction::Down;
    }
    return ShiftAction::None;
}

// ---------- OptimalDriver ----------
//
// Project the wheel force the car would produce in each gear at its
// current speed, and pick the gear with the greatest force. This is what
// a torque-curve-aware driver does on a drag strip.

ShiftAction OptimalDriver::shiftDecision(const DriverState& state,
                                         const Vehicle& vehicle) const {
    const auto& trans  = vehicle.transmission();
    const auto& engine = vehicle.engine();
    const auto& tire   = vehicle.tire();

    if (trans.numGears() == 0) return ShiftAction::None;

    auto wheelForceInGear = [&](std::size_t gearIdx) {
        double ratio = trans.totalRatio(gearIdx);
        // rpm = v / r * ratio * 60 / (2*pi)
        double rpm = state.speedMps / tire.radius() * ratio * 60.0 / (2.0 * M_PI);
        if (rpm > engine.redlineRpm()) return -1.0;  // invalid: would over-rev
        if (rpm < engine.idleRpm()) rpm = engine.idleRpm();
        double torque = engine.torqueAt(rpm);
        double wheelTorque = torque * ratio * trans.efficiency();
        return wheelTorque / tire.radius();
    };

    std::size_t cur = state.currentGear;
    double curForce = wheelForceInGear(cur);

    // Consider the gear above, if any.
    if (cur + 1 < trans.numGears()) {
        double upForce = wheelForceInGear(cur + 1);
        if (upForce > curForce * 1.02) {
            // Only upshift if materially better, to prevent hunting.
            return ShiftAction::Up;
        }
    }

    // Consider the gear below, if any.
    if (cur > 0) {
        double downForce = wheelForceInGear(cur - 1);
        // Downshift only if the current gear is invalid (over-rev) or
        // significantly worse than the lower gear.
        if (curForce < 0.0 || downForce > curForce * 1.05) {
            return ShiftAction::Down;
        }
    }

    // Safety: if current gear over-revs, always downshift.
    if (curForce < 0.0 && cur > 0) return ShiftAction::Down;

    return ShiftAction::None;
}

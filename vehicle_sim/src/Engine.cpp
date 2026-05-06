// Engine.cpp
#include "Engine.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <cmath>

Engine::Engine() : idle_(800.0), redline_(6500.0) {}

Engine::Engine(double idleRpm, double redlineRpm)
    : idle_(idleRpm), redline_(redlineRpm) {}

void Engine::addTorquePoint(double rpm, double torqueNm) {
    torqueCurve_.addPoint(rpm, torqueNm);
}

double Engine::torqueAt(double rpm) const {
    if (torqueCurve_.empty()) return 0.0;
    // Clamp RPM into a physically reasonable range before lookup.
    if (rpm < idle_)    rpm = idle_;
    if (rpm > redline_) rpm = redline_;
    return torqueCurve_.evaluate(rpm);
}

double Engine::peakTorqueRpm() const {
    if (torqueCurve_.empty()) return idle_;
    return torqueCurve_.argMax();
}

double Engine::peakPowerKw() const {
    // Power = torque * omega. Sweep a range of RPMs and find the max.
    if (torqueCurve_.empty()) return 0.0;
    double best = 0.0;
    for (double r = idle_; r <= redline_; r += 50.0) {
        double tq = torqueCurve_.evaluate(r);
        double omega = r * 2.0 * M_PI / 60.0;   // rad/s
        double powerW = tq * omega;
        if (powerW > best) best = powerW;
    }
    return best / 1000.0;
}

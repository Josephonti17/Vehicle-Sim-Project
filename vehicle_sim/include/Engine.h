// Engine.h
// Models an internal combustion engine via a torque curve (RPM -> Nm).
// Encapsulates redline and idle RPM.
#ifndef ENGINE_H
#define ENGINE_H

#include "Interpolator.h"
#include <string>

class Engine {
public:
    Engine();
    Engine(double idleRpm, double redlineRpm);

    // Add a single torque-curve data point (RPM, torque in Nm).
    void addTorquePoint(double rpm, double torqueNm);

    // Look up torque (Nm) at the given RPM.
    double torqueAt(double rpm) const;

    // RPM at which the torque curve reaches its peak value.
    double peakTorqueRpm() const;

    double idleRpm() const    { return idle_; }
    double redlineRpm() const { return redline_; }

    void setIdleRpm(double v)    { idle_ = v; }
    void setRedlineRpm(double v) { redline_ = v; }

    // Peak horsepower estimate in kW, computed from the torque curve.
    double peakPowerKw() const;

private:
    double idle_;
    double redline_;
    Interpolator<double> torqueCurve_;
};

#endif // ENGINE_H

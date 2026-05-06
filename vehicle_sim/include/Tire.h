// Tire.h
// Represents the tire/wheel interface between the drivetrain and the road.
// Holds rolling radius, rolling resistance coefficient, and peak grip (mu).
#ifndef TIRE_H
#define TIRE_H

class Tire {
public:
    Tire();
    Tire(double radiusM, double rollingResistance, double grip);

    double radius() const           { return radius_; }
    double rollingResistance() const { return crr_; }
    double grip() const             { return mu_; }

    void setRadius(double r)           { radius_ = r; }
    void setRollingResistance(double c) { crr_ = c; }
    void setGrip(double m)             { mu_ = m; }

private:
    double radius_;  // meters
    double crr_;     // rolling resistance coefficient (unitless)
    double mu_;      // peak grip coefficient (unitless)
};

#endif // TIRE_H

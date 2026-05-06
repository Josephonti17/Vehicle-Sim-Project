// Tire.cpp
#include "Tire.h"

Tire::Tire() : radius_(0.33), crr_(0.012), mu_(1.0) {}

Tire::Tire(double radiusM, double rollingResistance, double grip)
    : radius_(radiusM), crr_(rollingResistance), mu_(grip) {}

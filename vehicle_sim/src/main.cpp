// main.cpp
// Command-line interface for the Vehicle Performance Simulator.
//
// Supports:
//   * Loading vehicles from key=value config files
//   * Listing loaded vehicles
//   * Running 0-60 mph, quarter-mile, and top-speed tests
//   * Head-to-head racing over a selected distance
//   * Writing time-series CSV traces for plotting
//
// Demonstrates: std::map (vehicle registry), std::vector (trace, gear
// ratios), std::sort / std::accumulate / std::find_if, std::string parsing,
// smart pointers, templates (via Interpolator), polymorphism (Driver), and
// modular multi-file organization.

#include "Engine.h"
#include "Transmission.h"
#include "Tire.h"
#include "Driver.h"
#include "Vehicle.h"
#include "Simulation.h"
#include "Race.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace {

// Trim leading/trailing whitespace from a string (std::string processing).
std::string trim(const std::string& s) {
    auto a = s.find_first_not_of(" \t\r\n");
    auto b = s.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
}

// Split "a,b,c" into doubles. Demonstrates std::string and std::getline.
std::vector<double> parseDoubleList(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) out.push_back(std::stod(item));
    }
    return out;
}

// Load a Vehicle from a config file. Returns empty unique_ptr on failure.
// File format (one key=value per line, '#' starts comments):
//   name=2018 Mustang EcoBoost
//   mass=1664
//   cd=0.35
//   frontal_area=2.25
//   idle=800
//   redline=6500
//   torque=1000,300
//   torque=2500,475
//   ...
//   gears=4.236,2.538,1.665,1.238,1.000,0.756
//   final_drive=3.31
//   efficiency=0.85
//   tire_radius=0.333
//   rolling_resistance=0.012
//   grip=1.0
//   driver=optimal  (or "aggressive")
std::unique_ptr<Vehicle> loadVehicleFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Could not open " << path << "\n";
        return nullptr;
    }

    std::string name = "Unnamed";
    double mass = 1500, cd = 0.32, area = 2.2;
    double idle = 800, redline = 6500;
    std::vector<std::pair<double,double>> torquePoints;
    std::vector<double> gears;
    double finalDrive = 3.31, eff = 0.85;
    double tireRadius = 0.33, crr = 0.012, grip = 1.0;
    std::string driverType = "optimal";

    std::string line;
    while (std::getline(in, line)) {
        // Strip comments and trim whitespace.
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));

        try {
            if      (key == "name")              name = val;
            else if (key == "mass")              mass = std::stod(val);
            else if (key == "cd")                cd = std::stod(val);
            else if (key == "frontal_area")      area = std::stod(val);
            else if (key == "idle")              idle = std::stod(val);
            else if (key == "redline")           redline = std::stod(val);
            else if (key == "torque") {
                auto parts = parseDoubleList(val);
                if (parts.size() == 2)
                    torquePoints.emplace_back(parts[0], parts[1]);
            }
            else if (key == "gears")             gears = parseDoubleList(val);
            else if (key == "final_drive")       finalDrive = std::stod(val);
            else if (key == "efficiency")        eff = std::stod(val);
            else if (key == "tire_radius")       tireRadius = std::stod(val);
            else if (key == "rolling_resistance") crr = std::stod(val);
            else if (key == "grip")              grip = std::stod(val);
            else if (key == "driver")            driverType = val;
        } catch (const std::exception& e) {
            std::cerr << "Bad value on line: " << line << " (" << e.what() << ")\n";
        }
    }

    // Build the Engine.
    Engine engine(idle, redline);
    // std::sort the torque points by RPM (generic algorithm).
    std::sort(torquePoints.begin(), torquePoints.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& p : torquePoints) engine.addTorquePoint(p.first, p.second);

    Transmission trans(gears, finalDrive, eff);
    Tire tire(tireRadius, crr, grip);

    // Pick driver strategy polymorphically.
    std::unique_ptr<Driver> driver;
    if (driverType == "aggressive") {
        driver = std::make_unique<AggressiveDriver>();
    } else {
        driver = std::make_unique<OptimalDriver>();
    }

    auto vehicle = std::make_unique<Vehicle>(
        name, mass, cd, area,
        std::move(engine), std::move(trans), std::move(tire),
        std::move(driver));
    return vehicle;
}

// Pretty-print simulation results.
void printResults(const SimulationResults& r) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n--- Results for " << r.vehicleName << " ---\n";
    if (r.zeroToThirtyTime  >= 0) std::cout << "0-30 mph:    " << r.zeroToThirtyTime  << " s\n";
    if (r.zeroToSixtyTime   >= 0) std::cout << "0-60 mph:    " << r.zeroToSixtyTime   << " s\n";
    if (r.zeroToHundredTime >= 0) std::cout << "0-100 mph:   " << r.zeroToHundredTime << " s\n";
    if (r.quarterMileTime   >= 0) {
        std::cout << "1/4 mile:    " << r.quarterMileTime << " s @ "
                  << r.quarterMileTrapMph << " mph trap\n";
    }
    if (r.topSpeedMph > 0) std::cout << "Top speed:   " << r.topSpeedMph << " mph\n";
    std::cout << "Total time:  " << r.totalTime << " s\n";
    std::cout << "Distance:    " << r.totalDistance << " m\n";
}

int readMenuChoice(int lo, int hi) {
    int choice;
    while (true) {
        std::cout << "> ";
        if (std::cin >> choice && choice >= lo && choice <= hi) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return choice;
        }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Please enter a number between " << lo << " and " << hi << ".\n";
    }
}

std::string readLine(const std::string& prompt) {
    std::cout << prompt;
    std::string s;
    std::getline(std::cin, s);
    return trim(s);
}

} // namespace

int main() {
    std::cout << "=======================================================\n"
              << "  Vehicle Performance Simulator - EE 5103 Final Project\n"
              << "=======================================================\n\n";

    // std::map as the associative container requirement.
    std::map<std::string, std::unique_ptr<Vehicle>> garage;

    // Auto-load the bundled presets.
    const std::vector<std::string> presets = {
        "data/mustang_ecoboost.txt",
        "data/mustang_gt.txt",
        "data/miata.txt"
    };
    for (const auto& p : presets) {
        auto v = loadVehicleFromFile(p);
        if (v) {
            std::string name = v->name();
            garage[name] = std::move(v);
            std::cout << "Loaded: " << name << "\n";
        }
    }
    std::cout << "\n";

    Simulation sim(0.001);
    Race race(0.001);

    while (true) {
        std::cout << "\n--- Main Menu ---\n"
                  << "1. List garage\n"
                  << "2. Load vehicle from file\n"
                  << "3. Run 0-60 mph test\n"
                  << "4. Run quarter-mile test\n"
                  << "5. Run top-speed test\n"
                  << "6. Head-to-head race\n"
                  << "7. Exit\n";
        int c = readMenuChoice(1, 7);

        if (c == 7) break;

        if (c == 1) {
            if (garage.empty()) {
                std::cout << "(garage is empty)\n";
            } else {
                int i = 1;
                for (const auto& kv : garage) {
                    const Vehicle& v = *kv.second;
                    std::cout << i++ << ". " << v.name()
                              << "  mass=" << v.mass() << "kg"
                              << "  power=" << std::setprecision(0)
                              << v.engine().peakPowerKw() * 1.341 << " hp"
                              << "  driver=" << v.driver().name() << "\n";
                    std::cout << std::setprecision(2);
                }
            }
            continue;
        }

        if (c == 2) {
            std::string path = readLine("Path to config file: ");
            auto v = loadVehicleFromFile(path);
            if (v) {
                std::string name = v->name();
                garage[name] = std::move(v);
                std::cout << "Loaded " << name << "\n";
            }
            continue;
        }

        if (c == 6) {
            // Race: accept two or more names.
            if (garage.size() < 2) {
                std::cout << "Need at least two vehicles in the garage.\n";
                continue;
            }
            std::cout << "Enter vehicle names separated by semicolons.\n";
            std::cout << "Example: 2018 Mustang EcoBoost;2018 Mustang GT\n";
            std::string input = readLine("Vehicles: ");

            // Split by ';'.
            std::vector<std::string> names;
            std::stringstream ss(input);
            std::string tok;
            while (std::getline(ss, tok, ';')) {
                tok = trim(tok);
                if (!tok.empty()) names.push_back(tok);
            }
            if (names.size() < 2) {
                std::cout << "Need at least two valid names.\n";
                continue;
            }

            std::vector<Vehicle*> entrants;
            for (const auto& n : names) {
                // std::find_if on the map via an iterator range.
                auto it = garage.find(n);
                if (it == garage.end()) {
                    std::cout << "Unknown vehicle: " << n << "\n";
                } else {
                    entrants.push_back(it->second.get());
                }
            }
            if (entrants.size() < 2) continue;

            auto results = race.runFixedDistance(entrants, 402.336);
            std::cout << "\n--- Race results (1/4 mile) ---\n";
            int pos = 1;
            for (const auto& e : results.entries) {
                std::cout << pos++ << ". " << e.vehicleName
                          << "  " << std::fixed << std::setprecision(2)
                          << e.finishTime << " s @ "
                          << e.finishSpeedMph << " mph\n";
            }
            // Write one CSV per entrant.
            for (const auto& tr : results.traces) {
                std::string fname = "race_" + tr.vehicleName + ".csv";
                std::replace(fname.begin(), fname.end(), ' ', '_');
                Simulation::writeTraceCsv(tr, fname);
                std::cout << "Trace written: " << fname << "\n";
            }
            continue;
        }

        // Tests 3/4/5 all need a selected vehicle.
        if (garage.empty()) {
            std::cout << "Garage is empty.\n";
            continue;
        }

        std::string name = readLine("Vehicle name: ");
        auto it = garage.find(name);
        if (it == garage.end()) {
            std::cout << "Unknown vehicle: " << name << "\n";
            continue;
        }

        TestType test = TestType::ZeroToSixty;
        if      (c == 3) test = TestType::ZeroToSixty;
        else if (c == 4) test = TestType::QuarterMile;
        else if (c == 5) test = TestType::TopSpeed;

        auto results = sim.run(*it->second, test);
        printResults(results);

        // Average acceleration during the run via std::accumulate.
        if (!results.trace.empty()) {
            double sumAccel = std::accumulate(results.trace.begin(),
                                              results.trace.end(), 0.0,
                [](double acc, const TraceSample& s) {
                    return acc + s.accelMps2;
                });
            double avgAccel = sumAccel / static_cast<double>(results.trace.size());
            std::cout << "Avg accel:   " << avgAccel << " m/s^2\n";
        }

        // Offer CSV write.
        std::string write = readLine("Write trace CSV? (y/n): ");
        if (!write.empty() && (write[0] == 'y' || write[0] == 'Y')) {
            std::string fname = name + "_trace.csv";
            std::replace(fname.begin(), fname.end(), ' ', '_');
            if (Simulation::writeTraceCsv(results, fname))
                std::cout << "Wrote " << fname << "\n";
            else
                std::cout << "Failed to write " << fname << "\n";
        }
    }

    std::cout << "\nGoodbye.\n";
    return 0;
}

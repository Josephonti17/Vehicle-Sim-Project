# Vehicle Performance Simulator

**EE 5103 Final Project — Engineering Programming with C++**

A C++ vehicle performance simulator that models a car's drivetrain and
predicts real-world metrics like 0–60 mph time, quarter-mile ET and trap
speed, and top speed. Vehicles are defined by key-value config files so
you can compare drivetrain configurations without recompiling.

## Build

Requires a C++17 compiler (tested with `g++`) and `make`.

```bash
make            # builds ./vehicle_sim
make run        # builds and launches the CLI
make clean      # removes build artifacts and CSV output
```

## Run

```bash
./vehicle_sim
```

The program auto-loads the three presets in `data/` at startup, then
presents a menu:

```
1. List garage
2. Load vehicle from file
3. Run 0-60 mph test
4. Run quarter-mile test
5. Run top-speed test
6. Head-to-head race
7. Exit
```

Tests 3–5 ask for a vehicle name (e.g. `2018 Mustang GT`). After a run
you can optionally write a time-series CSV trace for plotting.

The head-to-head race option takes semicolon-separated names:

```
Vehicles: 2018 Mustang EcoBoost;2018 Mustang GT;2018 Mazda Miata
```

and produces one CSV per entrant.

## Project layout

```
vehicle_sim/
├── Makefile
├── README.md
├── include/
│   ├── Interpolator.h    Templated linear interpolator
│   ├── Engine.h          Torque curve + redline/idle
│   ├── Transmission.h    Gear ratios, final drive, shift state
│   ├── Tire.h            Radius, rolling resistance, grip
│   ├── Driver.h          Abstract driver + derived strategies
│   ├── Vehicle.h         Composes all of the above
│   ├── Simulation.h      Runs a single test and records a trace
│   └── Race.h            Lockstep head-to-head racing
├── src/
│   ├── Engine.cpp
│   ├── Transmission.cpp
│   ├── Tire.cpp
│   ├── Driver.cpp
│   ├── Vehicle.cpp
│   ├── Simulation.cpp
│   ├── Race.cpp
│   └── main.cpp          CLI and vehicle config loader
└── data/
    ├── mustang_ecoboost.txt
    ├── mustang_gt.txt
    └── miata.txt
```

## Physics model

Per time step (default dt = 1 ms), for each vehicle:

1. Engine RPM is computed from road speed and current gear ratio.
2. The driver decides whether to upshift, downshift, or hold.
3. Engine torque is looked up via linear interpolation of the torque curve.
4. Wheel torque = engine torque × gear ratio × final drive × drivetrain efficiency.
5. Wheel force = wheel torque / tire radius.
6. Force is capped at the traction limit (μ · m · g).
7. Aerodynamic drag (½ ρ Cd A v²) and rolling resistance (Crr · m · g) are subtracted.
8. Net force gives acceleration via F = ma.
9. Euler integration updates velocity and position.

## C++ concepts demonstrated

| Requirement | Where |
|---|---|
| Multiple source/header files | 8 class pairs + main |
| User-defined classes | `Engine`, `Transmission`, `Tire`, `Driver`, `Vehicle`, `Simulation`, `Race` |
| `std::vector` | Gear ratios, torque points, time-series traces, race entrants |
| `std::map` | Named vehicle registry in `main.cpp` |
| Generic algorithms | `std::sort` (race results, torque points), `std::lower_bound` (torque lookup), `std::accumulate` (average acceleration), `std::find_if` / `std::max_element`, `std::replace` |
| `std::string` | Config file parsing with `std::getline`, `std::stringstream` |
| Encapsulation | All classes use private state with public interfaces |
| Constructors / destructors | Every class; `Vehicle` uses a non-default destructor via `unique_ptr` |
| Polymorphism | `Driver` base class with `AggressiveDriver` and `OptimalDriver` overriding `shiftDecision()` |
| Dynamic memory (smart pointers) | `std::unique_ptr<Driver>` owned by each `Vehicle` |
| Templates | `Interpolator<T>` used by `Engine` for torque-curve lookup |
| Copy control | `Vehicle` explicitly deletes copy and defaults move |
| Modular design | Independent subsystems; headers include only what they need |

## Example session

```
$ ./vehicle_sim
Loaded: 2018 Mazda Miata
Loaded: 2018 Mustang EcoBoost
Loaded: 2018 Mustang GT

--- Main Menu ---
...
> 4
Vehicle name: 2018 Mustang EcoBoost

--- Results for 2018 Mustang EcoBoost ---
0-30 mph:    1.66 s
0-60 mph:    4.27 s
0-100 mph:   11.17 s
1/4 mile:    12.98 s @ 107.42 mph trap
Top speed:   107.42 mph
Total time:  12.98 s
Distance:    402.34 m
Avg accel:   3.67 m/s^2
```

## Config file format

Each preset is a plain-text file with `key=value` entries. Lines starting
with `#` are comments. Example:

```
name=My Car
mass=1500             # kg
cd=0.32               # drag coefficient
frontal_area=2.2      # m^2
idle=800              # RPM
redline=7000

# torque curve as (rpm, Nm) pairs, one per line
torque=1000,200
torque=3000,350
torque=5000,320

gears=4.2,2.5,1.7,1.2,1.0,0.8
final_drive=3.5
efficiency=0.85

tire_radius=0.33       # m
rolling_resistance=0.012
grip=0.9               # mu, peak coefficient of friction

driver=optimal         # "optimal" or "aggressive"
```

## Limitations and future work

The model intentionally stays simple. Known omissions that the final
report discusses in depth:

- No clutch slip or torque converter modeling at launch — makes very
  low-speed acceleration slightly optimistic.
- No tire slip curve — grip is a hard cap rather than a friction model.
- No rotational inertia of the drivetrain — ignores the energy required
  to spin up the flywheel, driveshaft, and wheels.
- No weight transfer during acceleration — affects which axle hits the
  traction limit first.
- No shift time — gear changes are instantaneous.
- Single-axle (RWD) assumption with no differential modeling.
- Euler integration; RK4 would be more accurate for longer runs.

Adding any of these would be a good extension for a follow-up project.

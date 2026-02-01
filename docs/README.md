# robot-2026-rewrite

**FRC Team 503 (Frog Force) — 2026 REBUILT Robot Code**

This is a ground-up rewrite of our robot code for the 2026 FIRST Robotics game **REBUILT**. Built with Java and WPILib.

## Overview

- **Language:** Java
- **Framework:** WPILib (FRC Control System)
- **Architecture:** AdvantageKit logging + modular subsystems
- **Status:** Active development for 2026 season

## Structure

```
├── src/main/java/org/frogforce503/
│   ├── robot/                 # Main robot logic
│   │   ├── RobotContainer.java    # Central config
│   │   ├── subsystems/            # Drive, turret, flywheels, etc.
│   │   ├── commands/              # Command-based control
│   │   ├── auto/                  # Autonomous routines
│   │   └── constants/             # Field, hardware, tuner constants
│   └── lib/                   # Reusable utilities
│       ├── math/              # Geometry & calculations
│       ├── subsystem/         # Base subsystem classes
│       ├── swerve/            # Swerve drive utilities
│       └── vision/            # AprilTag & object detection
├── vendordeps/                # WPILib + 3rd-party libraries
└── docs/                      # Documentation
```

## Key Features

- **Swerve Drivetrain:** CANivore + Kraken X60 motors
- **Vision:** AprilTag localization + object detection (PhotonVision)
- **Superstructure:** Turret, flywheels, hood, intake, indexer
- **Simulation:** MapleSim for physics-accurate testing

![Elastic Layout of Dashboard](../images/20260131_Elastic_Layout.png)

## Getting Started

1. Clone repo and open in VS Code
2. Run `./gradlew build` to compile
3. Deploy with WPILib extension or `./gradlew deploy`

## Development Notes

- Uses **AdvantageKit** for logged-replay testing
- Modular IO pattern for hardware abstraction (sim/real/replay)
- Follow existing subsystem structure when adding features

## Disclaimer

⚠️ **This is a personal/educational rewrite and is NOT the official FRC Team 503 repository.** It is maintained separately and independently from the actual Frog Force codebase.

**Note:** Team-specific assets (CAD models, proprietary designs, etc.) are intentionally excluded from this repository to comply with FIRST guidelines and protect team intellectual property. Only code and general software architecture are published here.
package org.frogforce503.lib.motorcontrol;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public record FFConfig(
    double kS,
    double kG,
    double kV,
    double kA
) {
    public FFConfig() {
        this(0.0, 0.0, 0.0, 0.0);
    }

    // From WPILib feedforwards
    public FFConfig(SimpleMotorFeedforward simpleFF) {
        this(simpleFF.getKs(), 0.0, simpleFF.getKv(), simpleFF.getKa());
    }

    public FFConfig(ElevatorFeedforward elevatorFF) {
        this(elevatorFF.getKs(), elevatorFF.getKg(), elevatorFF.getKv(), elevatorFF.getKa());
    }

    public FFConfig(ArmFeedforward armFF) {
        this(armFF.getKs(), armFF.getKg(), armFF.getKv(), armFF.getKa());
    }

    // To WPILib feedforwards
    public SimpleMotorFeedforward getSimpleMotorFF() {
        return new SimpleMotorFeedforward(kS(), kV(), kA());
    }

    public ArmFeedforward getArmFF() {
        return new ArmFeedforward(kS(), kG(), kV(), kA());
    }

    public ElevatorFeedforward getElevatorFF() {
        return new ElevatorFeedforward(kS(), kG(), kV(), kA());
    }
}
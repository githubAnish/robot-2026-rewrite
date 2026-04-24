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

    public SimpleMotorFeedforward getSimpleMotorFF() {
        return new SimpleMotorFeedforward(kS(), kV(), kA());
    }

    public ElevatorFeedforward getElevatorFF() {
        return new ElevatorFeedforward(kS(), kG(), kV(), kA());
    }

    public ArmFeedforward getArmFF() {
        return new ArmFeedforward(kS(), kG(), kV(), kA());
    }
}
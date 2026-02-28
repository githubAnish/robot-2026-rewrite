package org.frogforce503.robot.subsystems.climberhook.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.climberhook.ClimberHookConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberHookIOSim extends ClimberHookIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final ElevatorSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double simCarriageMass = Units.lbsToKilograms(16.895); // from CAD (Climber carriage (6.423 lb) + shoulder (3.000 lb) + arm (2.229 lb) + gripper (5.243 lb))

    public ClimberHookIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new ElevatorSim(
                motorModel,
                ClimberHookConstants.mechanismRatio,
                simCarriageMass,
                ClimberHookConstants.sprocketPitchDiameter / 2,
                ClimberHookConstants.minHeight,
                ClimberHookConstants.maxHeight,
                true,
                ClimberHookConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(ClimberHookConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(ClimberHookIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(physicsSim.getPositionMeters());
        motorSim.setVelocity(physicsSim.getVelocityMetersPerSecond());

        inputs.motorConnected = true;
        inputs.positionMeters = motorSim.getPosition();
        inputs.velocityMetersPerSec = motorSim.getVelocity();
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = motorSim.getMotorCurrent();
        inputs.tempCelsius = 24.0;
        inputs.limitSwitchPressed = motorSim.getPosition() == 0;
    }
}
package org.frogforce503.robot.subsystems.climberdeploy.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeployConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberDeployIOSim extends ClimberDeployIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final ElevatorSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double simCarriageMass = Units.lbsToKilograms(16.895); // from CAD (Climber carriage (6.423 lb) + shoulder (3.000 lb) + arm (2.229 lb) + gripper (5.243 lb))

    public ClimberDeployIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new ElevatorSim(
                motorModel,
                ClimberDeployConstants.mechanismRatio,
                simCarriageMass,
                ClimberDeployConstants.sprocketPitchDiameter / 2,
                ClimberDeployConstants.minHeight,
                ClimberDeployConstants.maxHeight,
                true,
                ClimberDeployConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(ClimberDeployConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(ClimberDeployIOInputs inputs) {
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
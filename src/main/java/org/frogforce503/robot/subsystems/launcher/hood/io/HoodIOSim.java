package org.frogforce503.robot.subsystems.launcher.hood.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim extends HoodIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNeo550(1);
    private final double length = Units.inchesToMeters(7.883352); // from CAD
    private final double moi = 0.0201162734731; // from CAD

    public HoodIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new SingleJointedArmSim(
                motorModel,
                HoodConstants.motorMechanismRatio,
                moi,
                length,
                HoodConstants.minAngle,
                HoodConstants.maxAngle,
                false,
                HoodConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(HoodConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(physicsSim.getAngleRads());
        motorSim.setVelocity(physicsSim.getVelocityRadPerSec());

        inputs.motorConnected = true;
        inputs.positionRad = motorSim.getPosition();
        inputs.velocityRadPerSec = motorSim.getVelocity();
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = motorSim.getMotorCurrent();
        inputs.tempCelsius = 24.0;
    }
}
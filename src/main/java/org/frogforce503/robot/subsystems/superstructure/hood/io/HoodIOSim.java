package org.frogforce503.robot.subsystems.superstructure.hood.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.HoodConfig;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim extends HoodIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double length = 0.33; // in meters, from https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/hood/HoodIOSim.java
    private final double moi = 0.004; // kg * m^2, from https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/hood/HoodIOSim.java

    public HoodIOSim() {
        final HoodConfig hoodConfig = Robot.bot.getHoodConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new SingleJointedArmSim(
                motorModel,
                hoodConfig.mechanismRatio(),
                moi,
                length,
                hoodConfig.minAngle(),
                hoodConfig.maxAngle(),
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

        inputs.data =
            new HoodIOData(
                true,
                motorSim.getPosition(),
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    }
}
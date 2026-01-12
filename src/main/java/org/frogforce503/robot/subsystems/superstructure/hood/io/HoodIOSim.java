package org.frogforce503.robot.subsystems.superstructure.hood.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.HoodConfig;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim extends HoodIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim hoodSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double length = Units.inchesToMeters(19);
    private final double moi = 0.85; // kg * m^2

    public HoodIOSim() {
        final HoodConfig hoodConfig = Robot.bot.getHoodConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        hoodSim =
            new SingleJointedArmSim(
                motorModel,
                hoodConfig.mechanismRatio(),
                moi,
                length,
                hoodConfig.minAngle(),
                hoodConfig.maxAngle(),
                true,
                HoodConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(HoodConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        hoodSim.setInputVoltage(appliedVolts);
        hoodSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(hoodSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(hoodSim.getAngleRads());
        motorSim.setVelocity(hoodSim.getVelocityRadPerSec());

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
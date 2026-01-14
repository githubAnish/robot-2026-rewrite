package org.frogforce503.robot.subsystems.superstructure.turret.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.TurretConfig;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim extends TurretIOSpark {
    // Control
    private final SparkSim motorSim;
    private final DCMotorSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.01;

    public TurretIOSim() {
        final TurretConfig turretConfig = Robot.bot.getTurretConfig();

        motorSim = new SparkSim(super.getMotor(), motorModel);
        physicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, turretConfig.mechanismRatio()), motorModel);

        // Sync physics and motor sim positions
        motorSim.setPosition(TurretConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(physicsSim.getAngularPositionRad());
        motorSim.setVelocity(physicsSim.getAngularVelocityRadPerSec());
        
        inputs.data =
            new TurretIOData(
                true,
                motorSim.getPosition(),
                motorSim.getPosition(), // assume relative & absolute encoders have same position
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    }
}
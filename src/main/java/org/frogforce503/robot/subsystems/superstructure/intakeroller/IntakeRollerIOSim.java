package org.frogforce503.robot.subsystems.superstructure.intakeroller;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.IntakeRollerConfig;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim extends IntakeRollerIOSpark {
    // Control
    private final SparkSim motorSim;
    private final DCMotorSim rollerSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.0001;

    public IntakeRollerIOSim() {
        final IntakeRollerConfig rollerConfig = Robot.bot.getIntakeRollerConfig();

        motorSim = new SparkSim(super.getMotor(), motorModel);
        rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, rollerConfig.mechanismRatio()), motorModel);

        // Sync physics and motor sim positions
        motorSim.setVelocity(IntakeRollerConstants.START);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        rollerSim.setInputVoltage(appliedVolts);
        rollerSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(rollerSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setVelocity(rollerSim.getAngularVelocityRadPerSec());
        
        inputs.data =
            new IntakeRollerIOData(
                true,
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    }
}
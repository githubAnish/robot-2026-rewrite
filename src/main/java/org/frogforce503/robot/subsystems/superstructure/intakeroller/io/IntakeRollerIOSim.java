package org.frogforce503.robot.subsystems.superstructure.intakeroller.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim extends IntakeRollerIOSpark {
    // Control
    private final SparkSim motorSim;
    private final FlywheelSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.001;

    public IntakeRollerIOSim() {
        motorSim = new SparkSim(super.getMotor(), motorModel);
        physicsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorModel, moi, IntakeRollerConstants.mechanismRatio),
                motorModel);

        // Sync physics and motor sim positions
        motorSim.setVelocity(IntakeRollerConstants.START);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setVelocity(physicsSim.getAngularVelocityRadPerSec());
        
        inputs.data =
            new IntakeRollerIOData(
                true,
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    }
}
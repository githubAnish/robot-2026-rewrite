package org.frogforce503.robot.subsystems.intakeroller.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRollerConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim extends IntakeRollerIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final FlywheelSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.00007543644656; // from CAD

    public IntakeRollerIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorModel, moi, IntakeRollerConstants.motorMechanismRatio),
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
        
        inputs.motorConnected = true;
        inputs.velocityRadPerSec = motorSim.getVelocity();
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = motorSim.getMotorCurrent();
        inputs.tempCelsius = 24.0;
    }
}
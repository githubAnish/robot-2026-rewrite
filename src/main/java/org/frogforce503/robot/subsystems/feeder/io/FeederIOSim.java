package org.frogforce503.robot.subsystems.feeder.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.feeder.FeederConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim extends FeederIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final FlywheelSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.0008156798616; // from CAD

    public FeederIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorModel, moi, FeederConstants.motorMechanismRatio),
                motorModel);

        // Sync physics and motor sim positions
        motorSim.setVelocity(FeederConstants.START);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
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
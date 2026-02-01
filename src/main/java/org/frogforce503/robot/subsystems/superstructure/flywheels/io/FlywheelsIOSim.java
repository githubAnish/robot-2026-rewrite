package org.frogforce503.robot.subsystems.superstructure.flywheels.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelsIOSim extends FlywheelsIOSpark {
    // Control
    private final SparkSim motorSim;
    private final FlywheelSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.025;

    public FlywheelsIOSim() {
        motorSim = new SparkSim(super.getMotor(), motorModel);
        physicsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorModel, moi, FlywheelsConstants.mechanismRatio),
                motorModel);

        // Sync physics and motor sim positions
        motorSim.setVelocity(FlywheelsConstants.START);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
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
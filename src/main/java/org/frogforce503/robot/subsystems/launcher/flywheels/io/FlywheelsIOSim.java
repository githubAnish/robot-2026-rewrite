package org.frogforce503.robot.subsystems.launcher.flywheels.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.launcher.flywheels.FlywheelsConstants;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelsIOSim extends FlywheelsIOSpark {
    // Control
    private final SparkFlexSim motorSim;
    private final FlywheelSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNeoVortex(2); // leader = 1 motor, follower = 1 motor, total 2 motors
    private final double moi = 0.00400419546112; // from CAD

    public FlywheelsIOSim() {
        motorSim = new SparkFlexSim(super.getLeader(), motorModel);
        physicsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorModel, moi, FlywheelsConstants.motorMechanismRatio),
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
        
        inputs.leaderConnected = true;
        inputs.leaderVelocityRadPerSec = motorSim.getVelocity();
        inputs.leaderAppliedVolts = appliedVolts;
        inputs.leaderStatorCurrentAmps = motorSim.getMotorCurrent();
        inputs.leaderTempCelsius = 24.0;

        inputs.followerConnected = true;
        inputs.followerVelocityRadPerSec = motorSim.getVelocity();
        inputs.followerAppliedVolts = appliedVolts;
        inputs.followerStatorCurrentAmps = motorSim.getMotorCurrent();
        inputs.followerTempCelsius = 24.0;
    }
}
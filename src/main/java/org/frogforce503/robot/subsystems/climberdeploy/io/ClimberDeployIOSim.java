package org.frogforce503.robot.subsystems.climberdeploy.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeployConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberDeployIOSim extends ClimberDeployIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double length = Units.inchesToMeters(6.0); // from CAD
    private final double moi = 0.126086426615; // from CAD

    public ClimberDeployIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new SingleJointedArmSim(
                motorModel,
                ClimberDeployConstants.mechanismRatio,
                moi,
                length,
                ClimberDeployConstants.minAngle,
                ClimberDeployConstants.maxAngle,
                true,
                ClimberDeployConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(ClimberDeployConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(ClimberDeployIOInputs inputs) {
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
        inputs.limitSwitchPressed = !super.getLimitSwitch().get();
    } 
}
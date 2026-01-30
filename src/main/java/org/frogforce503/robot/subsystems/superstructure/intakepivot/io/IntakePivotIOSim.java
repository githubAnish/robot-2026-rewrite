package org.frogforce503.robot.subsystems.superstructure.intakepivot.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim extends IntakePivotIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double length = Units.inchesToMeters(14.75); // TODO measure the length from the pivot point to the center of mass of the 4-bar intake
    private final double moi = 0.62; // kg * m^2, TODO measure the moi from the pivot point

    public IntakePivotIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new SingleJointedArmSim(
                motorModel,
                IntakePivotConstants.mechanismRatio,
                moi,
                length,
                IntakePivotConstants.minAngle,
                IntakePivotConstants.maxAngle,
                true,
                IntakePivotConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(IntakePivotConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(physicsSim.getAngleRads());
        motorSim.setVelocity(physicsSim.getVelocityRadPerSec());

        inputs.data =
            new IntakePivotIOData(
                true,
                motorSim.getPosition(),
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    } 
}
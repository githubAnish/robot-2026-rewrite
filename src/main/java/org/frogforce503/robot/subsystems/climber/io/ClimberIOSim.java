package org.frogforce503.robot.subsystems.climber.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim extends ClimberIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final ElevatorSim physicsSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double simCarriageMass = Units.lbsToKilograms(16.895); // from CAD (Climber carriage (6.423 lb) + shoulder (3.000 lb) + arm (2.229 lb) + gripper (5.243 lb))

    public ClimberIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        physicsSim =
            new ElevatorSim(
                motorModel,
                ClimberConstants.mechanismRatio,
                simCarriageMass,
                ClimberConstants.sprocketPitchDiameter / 2,
                ClimberConstants.minHeight,
                ClimberConstants.maxHeight,
                true,
                ClimberConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(ClimberConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(physicsSim.getPositionMeters());
        motorSim.setVelocity(physicsSim.getVelocityMetersPerSecond());

        inputs.data =
            new ClimberIOData(
                true,
                motorSim.getPosition(),
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0,
                motorSim.getPosition() == 0);
    }
}
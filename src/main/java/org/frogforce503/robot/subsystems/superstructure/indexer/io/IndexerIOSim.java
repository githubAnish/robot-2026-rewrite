package org.frogforce503.robot.subsystems.superstructure.indexer.io;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim extends IndexerIOSpark {
    // Control
    private final SparkSim motorSim;
    private final FlywheelSim physicsSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double moi = 0.001;

    public IndexerIOSim() {
        motorSim = new SparkSim(super.getMotor(), motorModel);
        physicsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorModel, moi, IndexerConstants.mechanismRatio),
                motorModel);

        // Sync physics and motor sim positions
        motorSim.setVelocity(IndexerConstants.START);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        physicsSim.setInputVoltage(appliedVolts);
        physicsSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(physicsSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setVelocity(physicsSim.getAngularVelocityRadPerSec());
        
        inputs.data =
            new IndexerIOData(
                true,
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    }
}
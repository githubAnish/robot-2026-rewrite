package org.frogforce503.robot.commands.tuning;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Measures the velocity feedforward constants (kS and kV) for the drive motors.
 * This command should only be used in voltage control mode.
 */
public class DriveFeedforwardCharacterization extends Command {
    private final Drive drive;

    private final double ffStartDelaySec = 2.0;
    private final double ffRampRateVoltsPerSec = 0.1;

    private final List<Double> velocitySamples = new LinkedList<>();
    private final List<Double> voltageSamples = new LinkedList<>();
    private final Timer timer = new Timer();

    private CharacterizationState currentState;

    private enum CharacterizationState {
        ALIGNING,
        CHARACTERIZING
    }

    public DriveFeedforwardCharacterization(Drive drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        velocitySamples.clear();
        voltageSamples.clear();

        currentState = CharacterizationState.ALIGNING;

        timer.restart();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case ALIGNING:
                drive.runCharacterization(0.0);
                
                if (timer.hasElapsed(ffStartDelaySec)) {
                    timer.restart();
                    currentState = CharacterizationState.CHARACTERIZING;
                }
                break;

            case CHARACTERIZING:
                double voltage = timer.get() * ffRampRateVoltsPerSec;

                drive.runCharacterization(voltage);

                velocitySamples.add(drive.getFFCharacterizationVelocityRotPerSec());
                voltageSamples.add(voltage);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runCharacterization(0.0);

        int n = velocitySamples.size();
        
        if (n < 2) {
            System.out.println("FF Characterization cancelled before enough data was collected.");
            return;
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;

        for (int i = 0; i < n; i++) {
            double v = velocitySamples.get(i);
            double u = voltageSamples.get(i);

            sumX += v;
            sumY += u;
            sumXY += v * u;
            sumX2 += v * v;
        }

        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

        NumberFormat formatter = new DecimalFormat("#0.00000");
        System.out.println("********** Drive FF Characterization Results **********");
        System.out.println("\tkS: " + formatter.format(kS));
        System.out.println("\tkV: " + formatter.format(kV));
    }
}
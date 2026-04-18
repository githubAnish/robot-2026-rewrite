package org.frogforce503.robot.commands.tuning;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Measures the robot's wheel radius by driving straight and comparing actual vs reported distance.
 * Measure the actual distance traveled using a measuring tape.
 * See https://www.frc5712.com/swerve-calibration.
 */
public class WheelRadiusCharacterizationDriveStraight extends Command {
    private final Drive drive;
    
    private final double driveSpeed = 0.5;

    private final WheelRadiusCharacterizationState charState = new WheelRadiusCharacterizationState();

    private final LoggedTunableNumber actualDistanceInches =
        new LoggedTunableNumber(
            "WheelRadiusCharacterizationDriveStraight/ActualDistanceInches",
            0.0);

    public WheelRadiusCharacterizationDriveStraight(Drive drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        charState.positions = drive.getWheelRadiusCharacterizationPositionsRad();
    }

    @Override
    public void execute() {
        drive.runVelocity(new ChassisSpeeds(driveSpeed, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        double[] endPositions = drive.getWheelRadiusCharacterizationPositionsRad();

        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(endPositions[i] - charState.positions[i]) / 4.0;
        }

        double actualDistance = Units.inchesToMeters(actualDistanceInches.get());

        if (actualDistance <= 0.0 || wheelDelta <= 0.0) {
            System.out.println("Wheel radius characterization failed: invalid distance or wheel delta");
            return;
        }

        double newWheelRadius = actualDistance / wheelDelta;

        NumberFormat formatter = new DecimalFormat("#0.000000");
        System.out.println("********** Wheel Radius Drive-Forward Results **********");
        System.out.println("\tActual Distance: " + formatter.format(actualDistance) + " meters");
        System.out.println("\tWheel Delta: "     + formatter.format(wheelDelta)      + " radians");
        System.out.println(
            "\tNew Wheel Radius: "
                + formatter.format(newWheelRadius)
                + " meters ("
                + formatter.format(Units.metersToInches(newWheelRadius))
                + " inches)");
    }
}
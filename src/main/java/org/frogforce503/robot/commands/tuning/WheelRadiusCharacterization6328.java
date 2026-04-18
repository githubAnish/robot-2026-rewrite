package org.frogforce503.robot.commands.tuning;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Measures the robot's wheel radius by spinning in a circle. */
public class WheelRadiusCharacterization6328 extends Command {
    private final Drive drive;

    private final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
    private final double wheelRadiusRampRate = 0.05; // Rad/Sec^2
    private final double orientDuration = 1.0;  // Secs

    private final SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
    private final Timer timer = new Timer();

    private CharacterizationState currentState;
    private final WheelRadiusCharacterizationState charState = new WheelRadiusCharacterizationState();

    private enum CharacterizationState {
        ORIENTING,
        MEASURING
    }

    public WheelRadiusCharacterization6328(Drive drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        limiter.reset(0.0);

        currentState = CharacterizationState.ORIENTING;

        timer.restart();
    }

    @Override
    public void execute() {
        double speed = limiter.calculate(wheelRadiusMaxVelocity);
        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));

        switch (currentState) {
            case ORIENTING:
                if (timer.hasElapsed(orientDuration)) {
                    charState.positions = drive.getWheelRadiusCharacterizationPositionsRad();
                    charState.lastAngle = drive.getGyroRotation();
                    charState.gyroDelta = 0.0;

                    currentState = CharacterizationState.MEASURING;
                }
                break;

            case MEASURING:
                Rotation2d gyroRotation = drive.getGyroRotation();

                charState.gyroDelta += Math.abs(gyroRotation.minus(charState.lastAngle).getRadians());
                charState.lastAngle = gyroRotation;

                double[] positions = drive.getWheelRadiusCharacterizationPositionsRad();
                double wheelDelta = computeWheelDelta(positions, charState.positions);
                double wheelRadius = (charState.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentState != CharacterizationState.MEASURING) {
            System.out.println("Wheel radius characterization cancelled before measurement began.");
            return;
        }

        double[] positions = drive.getWheelRadiusCharacterizationPositionsRad();
        double wheelDelta = computeWheelDelta(positions, charState.positions);
        double wheelRadius = (charState.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

        NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
        System.out.println("********** Wheel Radius Characterization Results **********");
        System.out.println("\tWheel Delta: " + formatter.format(wheelDelta)  + " radians");
        System.out.println("\tGyro Delta: " + formatter.format(charState.gyroDelta) + " radians");
        System.out.println(
            "\tWheel Radius: "
                + formatter.format(wheelRadius)
                + " meters, "
                + formatter.format(Units.metersToInches(wheelRadius))
                + " inches");
    }

    private double computeWheelDelta(double[] currentPositions, double[] startPositions) {
        double delta = 0.0;
        for (int i = 0; i < 4; i++) {
            delta += Math.abs(currentPositions[i] - startPositions[i]) / 4.0;
        }
        return delta;
    }
}
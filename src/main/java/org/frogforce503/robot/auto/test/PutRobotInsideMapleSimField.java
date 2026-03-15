package org.frogforce503.robot.auto.test;

import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * <p> Sim-only auto mode placing the robot at a valid pose inside the MapleSim field. </p>
 * <p> This is because Most MapleSim interactions require the robot to start within field boundaries. </p>
 */
public class PutRobotInsideMapleSimField implements AutoMode {
    private final Drive drive;
    private final Pose2d targetPose = new Pose2d(1.889, 4.002, Rotation2d.kZero);

    public PutRobotInsideMapleSimField(Drive drive) {
        this.drive = drive;
    }

    @Override
    public Command getCommand() {
        return Commands.runOnce(() -> drive.setPose(targetPose));
    }

    @Override
    public Pose2d[] getPoses() {
        return new Pose2d[] {targetPose};
    }
}
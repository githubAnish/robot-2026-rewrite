package org.frogforce503.lib.rebuilt.sim;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.util.Zone;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class BumpPhysicsSim {
    private final Drive drive;

    private final Zone redLeftBump = AllianceFlipUtil.flip(AllianceFlipUtil::apply, FieldConstants.Bump.blueLeft);
    private final Zone redRightBump = AllianceFlipUtil.flip(AllianceFlipUtil::apply, FieldConstants.Bump.blueRight);

    private final double gravity = 9.81;
    private final double bumpKickScalar = 0.2; // Tune this (1.0 = Perfect rigid bounce (lots of air), 0.0 = Magnetically glued to the ramp)

    private final double robotHalfLength = DriveConstants.bumperLength / 2.0; 
    private final double robotHalfWidth = DriveConstants.bumperWidth / 2.0;

    private double currentZ = 0.0;
    private double velocityZ = 0.0;

    public BumpPhysicsSim(Drive drive) {
        this.drive = drive;
    }

    public Pose3d update() {
        Rotation2d yaw = drive.getRotation();

        // 1. Sample 4 corners AND the center point
        Translation2d pos = drive.getPose().getTranslation();
        TerrainState center = getTerrainState(pos);
        TerrainState fl = getTerrainState(pos.plus(new Translation2d(robotHalfLength, robotHalfWidth).rotateBy(yaw)));
        TerrainState fr = getTerrainState(pos.plus(new Translation2d(robotHalfLength, -robotHalfWidth).rotateBy(yaw)));
        TerrainState bl = getTerrainState(pos.plus(new Translation2d(-robotHalfLength, robotHalfWidth).rotateBy(yaw)));
        TerrainState br = getTerrainState(pos.plus(new Translation2d(-robotHalfLength, -robotHalfWidth).rotateBy(yaw)));

        // 2. Natural Tilting (Pitch/Roll)
        double front_h = (fl.height() + fr.height()) / 2.0;
        double back_h = (bl.height() + br.height()) / 2.0;
        double left_h = (fl.height() + bl.height()) / 2.0;
        double right_h = (fr.height() + br.height()) / 2.0;

        double pitch = Math.atan2(back_h - front_h, robotHalfLength * 2.0);
        double roll = Math.atan2(left_h - right_h, robotHalfWidth * 2.0);

        // 3. Rigid Body Target Z
        double dz_pitch = Math.sin(pitch) * robotHalfLength;
        double dz_roll = Math.sin(roll) * robotHalfWidth;

        double fl_z_offset = -dz_pitch + dz_roll;
        double fr_z_offset = -dz_pitch - dz_roll;
        double bl_z_offset =  dz_pitch + dz_roll;
        double br_z_offset =  dz_pitch - dz_roll;

        double targetZ =
            Math.max(
                center.height(),
                Math.max(
                    Math.max(fl.height() - fl_z_offset, fr.height() - fr_z_offset),
                    Math.max(bl.height() - bl_z_offset, br.height() - br_z_offset)
                ));

        // 4. Needed Vertical Velocity for "Kick" (Now with dampening!)
        double neededVelocityZ =
            (drive.getFieldVelocity().vxMetersPerSecond * center.slopeX()) + 
            (drive.getFieldVelocity().vyMetersPerSecond * center.slopeY());
        
        // Apply the dampener to simulate tire squish and energy loss
        neededVelocityZ *= bumpKickScalar;

        // 5. Ballistic Physics (Flight Logic)
        velocityZ -= gravity * Constants.loopPeriodSecs;
        currentZ += velocityZ * Constants.loopPeriodSecs;

        // Ground Collision 
        if (currentZ <= targetZ) {
            currentZ = targetZ;
            velocityZ = Math.max(velocityZ, neededVelocityZ);
        }

        return new Pose3d(
            drive.getPose().getX(),
            drive.getPose().getY(),
            currentZ, 
            new Rotation3d(roll, pitch, yaw.getRadians()));
    }

    private TerrainState getTerrainState(Translation2d pos) {
        if (!onBump(pos)) {
            return new TerrainState(0, 0, 0);
        }

        // Constants from Field Manual
        final double bumpDepth = Units.inchesToMeters(44.4);
        final double tan15 = Math.tan(Units.degreesToRadians(15.0));

        double distFromInit;
        double slopeDir;

        if (pos.getX() < FieldConstants.fieldLength / 2.0) {
            distFromInit = pos.getX() - FieldConstants.Lines.blueInitLineX;
            slopeDir = 1.0;

        } else {
            double redInitLineX = FieldConstants.fieldLength - FieldConstants.Lines.blueInitLineX;
            distFromInit = redInitLineX - pos.getX();
            slopeDir = -1.0;
        }

        double x = Math.max(0, Math.min(distFromInit, bumpDepth));

        if (x < bumpDepth / 2.0) {
            return new TerrainState(x * tan15, tan15 * slopeDir, 0);
        } else {
            return new TerrainState((bumpDepth - x) * tan15, -tan15 * slopeDir, 0);
        }
    }

    private boolean onBump(Translation2d pos) {
        return
            FieldConstants.Bump.blueLeft.contains(pos) ||
            FieldConstants.Bump.blueRight.contains(pos) ||
            redLeftBump.contains(pos) ||
            redRightBump.contains(pos);
    }

    private record TerrainState(
        double height,
        double slopeX,
        double slopeY) {}
}
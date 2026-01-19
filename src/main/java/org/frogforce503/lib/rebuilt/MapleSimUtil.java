package org.frogforce503.lib.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.frogforce503.robot.constants.field.FieldConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public final class MapleSimUtil {
    private MapleSimUtil() {}
    
    public static void scoreFuelIntoHub(
        Pose2d robotPose,
        ChassisSpeeds robotFieldRelativeVelocity,
        double turretFieldRelativeAngleRad,
        double hoodAngleRad
    ) {
        var x =
            SimulatedArena.getInstance()
                .getGamePiecesByType("Fuel")
                .stream()
                .filter(g -> !g.isGrounded())
                .map(GamePiece::getPose3d)
                .toArray(Pose3d[]::new);
                
        Logger.recordOutput("SuperstructureViz/3D/AllMovingFuel", x);

        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    new Translation2d(),
                    robotFieldRelativeVelocity,
                    robotPose.getRotation(), // need to change to field-relative turret angle
                    Inches.of(24),
                    MetersPerSecond.of(10),
                    Radians.of(hoodAngleRad))
                .withTargetPosition(() -> FieldConstants.Hub.blueShotPose)
                .withTargetTolerance(new Translation3d(0.5, 0.5, 0.5))
                .withProjectileTrajectoryDisplayCallBack(
                    pose3ds -> Logger.recordOutput("SuperstructureViz/3D/FuelShot", pose3ds.toArray(Pose3d[]::new)),
                    pose3ds -> Logger.recordOutput("SuperstructureViz/3D/UnsucessfulFuelShot", pose3ds.toArray(Pose3d[]::new))
                )
                .enableBecomesGamePieceOnFieldAfterTouchGround()
            );
    }
}
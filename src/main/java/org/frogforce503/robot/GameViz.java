package org.frogforce503.robot;

import java.util.Arrays;

import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.SuperstructureViz;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;

/** Simulates the field, including interaction with & movement of game elements. Uses physics simulation. */
public class GameViz {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final Hood hood;
    private final Flywheels flywheels;
    private final Climber climber;

    private final VisionSimulator visionViz;
    private final SuperstructureViz superstructureViz = new SuperstructureViz();

    private IntakeSimulation intakeSimulation;

    @Setter private double robotHeightMeters = 0.0;

    // Shoot Sim Constants
    private final double shooterFireRateBallsPerSec = 7; // How many balls can shooter fire within 1 sec?
    private final Timer shotTimer = new Timer();

    // Climb Sim Constants
    private final double climbRateScalarMetersPerSec = 44.5;
    private final Timer climbTimer = new Timer();
    
    public GameViz(
        Drive drive,
        IntakePivot intakePivot,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        VisionSimulator visionViz
    ) {
        this.drive = drive;
        this.intakePivot = intakePivot;
        this.hood = hood;
        this.flywheels = flywheels;
        this.climber = climber;
        
        this.visionViz = visionViz;

        if (RobotBase.isSimulation()) {
            intakeSimulation = MapleSimUtil.createIntake(drive.getMapleSimDrive().mapleSimDrive);
            
            // Fill preload fuel
            for (int i = 0; i < 8; i++) {
                intakeSimulation.addGamePieceToIntake();
            }
        }
    }

    public void update() {
        Pose3d drivePose3d =
            new Pose3d(drive.getPose())
                .plus(new Transform3d(0.0, 0.0, robotHeightMeters, Rotation3d.kZero));

        visionViz.update(drive.getPose());
        
        superstructureViz.update(
            drivePose3d,
            hood.getAngleRad(),
            intakePivot.getAngleRad(),
            climber.getHeightMeters());

        Translation3d[] fuelInHopper =
            MapleSimUtil.visualizeFuelInHopper(drivePose3d, intakeSimulation.getGamePiecesAmount());

        Translation3d[] fuelTranslations = // Convert fuel poses to translations to lower data processed by NT
            Arrays
                .stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")) // Get all fuel from MapleSim arena
                .map(Pose3d::getTranslation)
                .toArray(Translation3d[]::new);

        Logger.recordOutput("GameViz/DrivePose3d", drivePose3d);
        Logger.recordOutput("GameViz/FuelTranslations", fuelTranslations);
        Logger.recordOutput("GameViz/NumFuelInRobot", intakeSimulation.getGamePiecesAmount());
        Logger.recordOutput("GameViz/FuelInHopper", fuelInHopper);
    }

    public void startIntake() {
        intakeSimulation.startIntake();
    }

    public void stopIntake() {
        intakeSimulation.stopIntake();
    }

    public void shootFuel(boolean needFuelFromIntakeForShoot) {
        if (needFuelFromIntakeForShoot && intakeSimulation.getGamePiecesAmount() <= 0) {
            return; // Don't shoot balls if there are none
        }

        double shotRateBallsPerSec = shooterFireRateBallsPerSec;
        double shotDelaySec = 1.0 / shotRateBallsPerSec;

        // Allow very first shot (timer not used yet, get() == 0.0), or when cooldown has elapsed
        if (shotTimer.isRunning() && !shotTimer.hasElapsed(shotDelaySec)) {
            return; // Cooldown not done; skip creating new projectile
        }

        // Index fuel
        intakeSimulation.obtainGamePieceFromIntake();

        // Shoot fuel
        MapleSimUtil.createFuelProjectiles(
            drive.getPose(),
            drive.getFieldVelocity(),
            hood.getAngleRad(),
            flywheels.getVelocityRadPerSec());

        // Restart cooldown timer after firing
        shotTimer.restart();
    }

    public void startClimb() {
        climbTimer.restart();
    }

    public void climb() {
        // Scale climber velocity to restrict robot height & climbing speed to tower
        robotHeightMeters += climber.getVelocityMetersPerSec() / climbRateScalarMetersPerSec * climbTimer.get();
    }

    public void stopClimb() {
        climbTimer.stop();
        climbTimer.reset();
    }
}
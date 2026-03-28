package org.frogforce503.robot;

import java.util.Arrays;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.rebuilt.BumpPhysicsSim;
import org.frogforce503.lib.rebuilt.HubShiftUtil;
import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** Simulates the field, including interaction with & movement of game elements. Uses physics simulation. */
public class GameViz {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final Hood hood;
    private final Flywheels flywheels;
    private final Climber climber;

    private final VisionSimulator visionViz;
    private final SuperstructureViz superstructureViz = new SuperstructureViz();
    private final BumpPhysicsSim bumpSim = new BumpPhysicsSim();

    private IntakeSimulation intakeSimulation;

    // Arena Constants
    private final double outpostDumpThresholdDist = Units.inchesToMeters(6);
    private double robotClimbHeightMeters = 0.0;
    private int fuelShotInMatch = 0;

    // Shoot Sim Constants
    private final Timer shotTimer = new Timer();

    // Climb Sim Constants
    private final double climbRateScalarMetersPerSec = 1.0 / 200.0;
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
        // Get inputs
        Pose2d drivePose = drive.getPose();
        double distanceToOutpost =
            drivePose
                .getTranslation()
                .getDistance(AllianceFlipUtil.apply(FieldConstants.Outpost.blue).getTranslation());

        if (distanceToOutpost < outpostDumpThresholdDist) {
            MapleSimUtil.dumpFromOutpost();
        }

        // Apply bump physics
        Pose3d drivePose3d =
            bumpSim.update(drivePose, drive.getFieldVelocity(), Constants.loopPeriodSecs);

        // Add robot climb height
        drivePose3d = drivePose3d.plus(new Transform3d(0, 0, robotClimbHeightMeters, Rotation3d.kZero));

        // Update visualizers
        visionViz.update(drivePose);
        superstructureViz.update(drivePose3d, hood.getAngleRad(), intakePivot.getAngleRad());

        // Visualize fuel
        Translation3d[] fuelInHopper =
            MapleSimUtil.visualizeFuelInHopper(drivePose3d, intakeSimulation.getGamePiecesAmount());

        Translation3d[] fuelTranslations = // Convert fuel poses to translations to lower data processed by NT
            Arrays
                .stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")) // Get all fuel from MapleSim arena
                .map(Pose3d::getTranslation)
                .toArray(Translation3d[]::new);

        // Log data
        Logger.recordOutput("GameViz/DrivePose3d", drivePose3d);
        Logger.recordOutput("GameViz/FuelTranslations", fuelTranslations);
        Logger.recordOutput("GameViz/NumFuelInRobot", intakeSimulation.getGamePiecesAmount());
        Logger.recordOutput("GameViz/FuelInHopper", fuelInHopper);
        Logger.recordOutput("GameViz/Fuel Shot In Match", fuelShotInMatch);

        Logger.recordOutput(
            "GameViz/Remaining Shift Time",
            String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));

        Logger.recordOutput(
            "GameViz/Current Shift",
            HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    }

    public void startIntake() {
        intakeSimulation.startIntake();
    }

    public void stopIntake() {
        intakeSimulation.stopIntake();
    }

    public void shootFuel(boolean needFuelFromIntakeForShoot) {
        MapleSimUtil.shootFuel(
            drive.getPose(),
            drive.getFieldVelocity(),
            hood.getAngleRad(),
            flywheels.getVelocityRadPerSec(), 
            intakeSimulation,
            shotTimer,
            needFuelFromIntakeForShoot,
            () -> fuelShotInMatch++);
    }

    public void startClimb() {
        climbTimer.restart();
    }

    public void climb() {
        // Scale climber velocity to restrict robot height & climbing speed to tower
        robotClimbHeightMeters += -climber.getVelocityMetersPerSec() * climbRateScalarMetersPerSec * climbTimer.get();
    }

    public void stopClimb() {
        climbTimer.stop();
        climbTimer.reset();
    }
}
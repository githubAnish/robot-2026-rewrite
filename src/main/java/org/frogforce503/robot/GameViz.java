package org.frogforce503.robot;

import java.util.Arrays;

import org.frogforce503.lib.rebuilt.BumpPhysicsSim;
import org.frogforce503.lib.rebuilt.HubShiftUtil;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private double robotClimbHeightMeters = 0.0;

    // Arena Constants
    private int fuelShotInMatch = 0;

    // Shoot Sim Constants
    private final double leftMostFuelPositionOffset = Units.inchesToMeters(-8);
    private final double rightMostFuelPositionOffset = Units.inchesToMeters(10);
    private final double fuelReleasedPerShot = 4; // How many balls are fired at once?
    private final double shooterFireRateBallsPerSec = 7; // How many balls can shooter fire within 1 sec?
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
        // Apply bump physics
        Pose3d terrainPose =
            bumpSim.update(drive.getPose(), drive.getFieldVelocity(), Constants.loopPeriodSecs);

        // Add robot climb height
        Pose3d drivePose3d =
            terrainPose.plus(new Transform3d(0, 0, robotClimbHeightMeters, Rotation3d.kZero));

        // Update visualizers
        visionViz.update(drive.getPose());
        
        superstructureViz.update(
            drivePose3d,
            hood.getAngleRad(),
            intakePivot.getAngleRad());

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
            "Shifts/Remaining Shift Time",
            String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));

        Logger.recordOutput("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());

        Logger.recordOutput(
            "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());

        Logger.recordOutput(
            "Shifts/Active First?",
            DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

        Logger.recordOutput("GameViz/HubShiftOfficial", HubShiftUtil.getOfficialShiftInfo());
        Logger.recordOutput("GameViz/HubShiftShifted", HubShiftUtil.getShiftedShiftInfo());
    }

    public void startIntake() {
        intakeSimulation.startIntake();
    }

    public void stopIntake() {
        intakeSimulation.stopIntake();
    }

    private int computeFuelToShoot(int available) {
        if (available <= 0) {
            return 0;
        }

        double fillRatio = (double) available / 40.0; // Normalize (0 → 1)
        double curvedFill = Math.pow(fillRatio, 0.7); // Smooth curve
        double scaledMax = fuelReleasedPerShot * curvedFill; // Scale burst size

        // Bounds
        int minShot = Math.max(1, (int) Math.floor(scaledMax * 0.5));
        int maxShot = Math.max(1, (int) Math.ceil(scaledMax));

        // Weighted randomness
        double bias = curvedFill;
        double rand = Math.random();
        double weightedRand = (rand * (1 - bias)) + (Math.pow(rand, 0.5) * bias);

        int fuelToShoot = minShot + (int)(weightedRand * (maxShot - minShot + 1));

        // Simulate indexing inconsistency
        double misfeedChance = 0.15 * (1.0 - fillRatio); // Misfeed
        if (Math.random() < misfeedChance) {
            fuelToShoot -= 1;
        }

        double doubleFeedChance = 0.08 * fillRatio; // Double feed
        if (Math.random() < doubleFeedChance) {
            fuelToShoot += 1;
        }

        double stutterChance = 0.1; // Stutter
        if (Math.random() < stutterChance) {
            fuelToShoot += Math.random() < 0.5 ? -1 : 1;
        }

        return MathUtil.clamp(fuelToShoot, 1, available);
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

        // Check fuel amount
        int available = intakeSimulation.getGamePiecesAmount();
        int fuelToShoot = computeFuelToShoot(available);

        // Index fuel
        for (int i = 0; i < fuelToShoot; i++) {
            intakeSimulation.obtainGamePieceFromIntake();
        }

        // Shoot fuel
        double step = (fuelToShoot > 1) ? (rightMostFuelPositionOffset - leftMostFuelPositionOffset) / (fuelToShoot - 1) : 0.0;

        for (int i = 0; i < fuelToShoot; i++) {
            double offset = (fuelToShoot == 1) ? 0.0 : leftMostFuelPositionOffset + i * step;

            MapleSimUtil.createFuelProjectile(
                drive.getPose(),
                drive.getFieldVelocity(),
                hood.getAngleRad(),
                flywheels.getVelocityRadPerSec(),
                new Transform2d(0.0, offset, Rotation2d.kZero),
                () -> fuelShotInMatch++);
        }

        // Restart cooldown timer after firing
        shotTimer.restart();
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
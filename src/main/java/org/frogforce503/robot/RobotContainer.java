package org.frogforce503.robot;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.robot.Constants.Mode;
import org.frogforce503.robot.auto.AutoChooser;
import org.frogforce503.robot.auto.WarmupExecutor;
import org.frogforce503.robot.commands.AimAndPrepShot;
import org.frogforce503.robot.commands.AlignToClimb;
import org.frogforce503.robot.commands.EjectFuelFromIntake;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.LowerClimber;
import org.frogforce503.robot.commands.RaiseClimber;
import org.frogforce503.robot.commands.PutIntakeUpForShoot;
import org.frogforce503.robot.commands.ShootFuel;
import org.frogforce503.robot.commands.TeleopDriveCommand;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.io.ClimberIO;
import org.frogforce503.robot.subsystems.climber.io.ClimberIOSim;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOBasicSim;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
import org.frogforce503.robot.subsystems.feeder.Feeder;
import org.frogforce503.robot.subsystems.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.feeder.io.FeederIO;
import org.frogforce503.robot.subsystems.feeder.io.FeederIOSim;
import org.frogforce503.robot.subsystems.indexer.Indexer;
import org.frogforce503.robot.subsystems.indexer.io.IndexerIO;
import org.frogforce503.robot.subsystems.indexer.io.IndexerIOSim;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakepivot.io.IntakePivotIO;
import org.frogforce503.robot.subsystems.intakepivot.io.IntakePivotIOSim;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.intakeroller.io.IntakeRollerIO;
import org.frogforce503.robot.subsystems.intakeroller.io.IntakeRollerIOSim;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator;
import org.frogforce503.robot.subsystems.launcher.LaunchPreset;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.launcher.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.launcher.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.launcher.flywheels.io.FlywheelsIO;
import org.frogforce503.robot.subsystems.launcher.flywheels.io.FlywheelsIOSim;
import org.frogforce503.robot.subsystems.launcher.hood.Hood;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;
import org.frogforce503.robot.subsystems.launcher.hood.io.HoodIO;
import org.frogforce503.robot.subsystems.launcher.hood.io.HoodIOSim;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.VisionEstimateConsumer;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonSim;
import org.frogforce503.robot.viz.GameViz;
import org.frogforce503.robot.viz.PracticeMatchViz;
import org.frogforce503.robot.viz.VisionSimulator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Main container for robot subsystems, commands, and controller bindings.
 * Use https://www.padcrafter.com to visualize the controller bindings.
 */
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Vision vision;
    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;
    private Indexer indexer;
    private Feeder feeder;
    private Hood hood;
    private Flywheels flywheels;
    private Climber climber;

    // Sim
    private final GameViz gameViz;
    private final VisionSimulator visionViz = new VisionSimulator();

    // Auto
    private final AutoChooser autoChooser;
    private final WarmupExecutor warmupExecutor;

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);

    // Main Buttons
    private final Trigger intakeGround = driverXbox.leftTrigger();
    private final Trigger ejectIntake = driverXbox.leftBumper();
    
    private final Trigger shootFuel = driverXbox.rightTrigger();
    private final Trigger aimAndPrepShot = driverXbox.rightBumper();

    private final Trigger setBatterPreset = driverXbox.y();
    private final Trigger setTrenchPreset = driverXbox.x();
    private final Trigger setDepotPreset = driverXbox.a();

    private final Trigger climb = driverXbox.b();

    // Overrides
    private final Trigger toggleSlowMode = driverXbox.back();
    private final Trigger toggleRobotRelative = driverXbox.start();
    private final Trigger resetRobotRotation = driverXbox.povUp();
    private final Trigger xWheels = driverXbox.povDown();
    private final Trigger alignToClimb = driverXbox.povRight();

    // Commands
    private final TeleopDriveCommand teleopDriveCommand;
    
    // Other
    private final VisionEstimateConsumer visionEstimateConsumer = drive::addVisionMeasurement;

    public RobotContainer() {
        // Initialize subsystems based on robot type
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case CompBot -> {
                    // Not implemented
                }
                case PracticeBot -> {
                    // Not implemented
                }
                case ProgrammingBot -> {
                    // Not implemented
                }
                case SimBot -> {
                    drive =
                        new Drive(
                            Constants.usingMapleSim
                                ? new DriveIOMapleSim()
                                : new DriveIOBasicSim());

                    vision =
                        new Vision(
                            visionEstimateConsumer,
                            new AprilTagIO[] {
                                new AprilTagIOPhotonSim(VisionConstants.leftCameraConfig),
                                new AprilTagIOPhotonSim(VisionConstants.rightCameraConfig),
                                new AprilTagIOPhotonSim(VisionConstants.backCameraConfig),
                                new AprilTagIOPhotonSim(VisionConstants.backLeftCameraConfig)
                            }
                        );

                    intakePivot = new IntakePivot(new IntakePivotIOSim());
                    intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                    indexer = new Indexer(new IndexerIOSim());
                    feeder = new Feeder(new FeederIOSim());
                    hood = new Hood(new HoodIOSim());
                    flywheels = new Flywheels(new FlywheelsIOSim());

                    climber = new Climber(new ClimberIOSim());
                }
            }
        }

        // No-op implementations if replay or not defined above
        if (drive == null) {
            drive = new Drive(new DriveIO() {});
        }

        if (vision == null) {
            vision = new Vision(visionEstimateConsumer, new AprilTagIO[] {});
        }

        if (intakePivot == null) {
            intakePivot = new IntakePivot(new IntakePivotIO() {});
        }
        
        if (intakeRoller == null) {
            intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        }

        if (indexer == null) {
            indexer = new Indexer(new IndexerIO() {});
        }

        if (feeder == null) {
            feeder = new Feeder(new FeederIO() {});
        }
            
        if (hood == null) {
            hood = new Hood(new HoodIO() {});
        }

        if (flywheels == null) {
            flywheels = new Flywheels(new FlywheelsIO() {});
        }

        if (climber == null) {
            climber = new Climber(new ClimberIO() {});
        }

        // Create sim requirements
        gameViz =
            Constants.isPracticeMatch
                ? new PracticeMatchViz(drive, intakePivot, intakeRoller, hood, flywheels, climber, visionViz)
                : new GameViz(drive, intakePivot, intakeRoller, hood, flywheels, climber, visionViz);
        
        drive.setTrenchCollisionSim(gameViz.getTrenchCollisionSim());

        // Create auto requirements
        autoChooser = new AutoChooser(drive, intakePivot, intakeRoller, indexer, feeder, hood, flywheels, climber, gameViz);
        warmupExecutor = new WarmupExecutor(drive, autoChooser.getBlineAutoBuilder());

        // Initialize commands
        teleopDriveCommand = new TeleopDriveCommand(drive, driverXbox);

        // Configure default commands
        drive.setDefaultCommand(teleopDriveCommand);
        
        feeder.setDefaultCommand(
            Commands.runOnce((() -> feeder.setVelocity(FeederConstants.IDLE)), feeder)
                .withName("Feeder Default Command"));

        hood.setDefaultCommand(
            Commands.runOnce(() -> hood.setAngle(HoodConstants.DUCK_UNDER_TRENCH, 0.0), hood)
                .withName("Hood Default Command"));

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Bind main controls
        intakeGround
            .whileTrue(new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz));

        shootFuel
            .whileTrue(new ShootFuel(indexer, gameViz))
            .and(intakeGround.negate())
            .whileTrue(new PutIntakeUpForShoot(intakePivot, intakeRoller, gameViz));

        aimAndPrepShot
            .whileTrue(new AimAndPrepShot(drive, feeder, hood, flywheels, driverXbox));

        ejectIntake
            .whileTrue(new EjectFuelFromIntake(intakePivot, intakeRoller, indexer, feeder));

        bindShotPreset(setBatterPreset, LaunchPreset.BATTER);
        bindShotPreset(setTrenchPreset, LaunchPreset.TRENCH);
        bindShotPreset(setDepotPreset, LaunchPreset.TOWER);

        climb
            .onTrue(new RaiseClimber(climber))
            .onFalse(new LowerClimber(climber, gameViz));

        // Bind override controls
        toggleSlowMode
            .onTrue(
                Commands.runOnce(teleopDriveCommand::toggleSlowMode)
                    .withName("Toggling Slow Mode"));

        toggleRobotRelative
            .onTrue(
                Commands.runOnce(teleopDriveCommand::toggleRobotRelative)
                    .withName("Toggling Robot Relative Mode"));

        resetRobotRotation
            .onTrue(
                Commands.runOnce(() -> drive.setAngle(AllianceFlipUtil.apply(Rotation2d.kZero)))
                    .withName("Reset Robot Rotation"));

        xWheels
            .onTrue(
                Commands.runOnce(drive::stopWithX)
                    .withName("Stop With X"));

        alignToClimb
            .whileTrue(new AlignToClimb(drive));
    }

    private void bindShotPreset(Trigger trigger, LaunchPreset shotPreset) {
        trigger
            .onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance().setShotPreset(shotPreset)))
            .onFalse(Commands.runOnce(() -> LaunchCalculator.getInstance().setShotPreset(LaunchPreset.NONE)));
    }

    public void robotPeriodic() {
        // Calculate latest shot info
        ShotInfo shotInfo =
            LaunchCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        // Check if shot feasible
        boolean shotDistanceValid =
            LaunchCalculator.inAllianceZone(drive.getPose())
                ? MathUtils.inRange(shotInfo.launcherToTargetDistance(), LaunchCalculator.minDistanceHubShoot, LaunchCalculator.maxDistanceHubShoot)
                : MathUtils.inRange(shotInfo.launcherToTargetDistance(), LaunchCalculator.minDistanceLobShoot, LaunchCalculator.maxDistanceLobShoot);
                
        boolean driveAtGoal = MathUtil.isNear(shotInfo.driveAngle().getRadians(), drive.getRotation().getRadians(), DriveConstants.aimTolerance);
        boolean hoodAtGoal = hood.isAtAngle(shotInfo.hoodAngleRad(), HoodConstants.shootOnMoveTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(shotInfo.flywheelsVelocityRadPerSec(), FlywheelsConstants.tolerance);

        boolean isCalculatedShotFeasible =
            shotDistanceValid && driveAtGoal && hoodAtGoal && flywheelsAtGoal;

        LaunchCalculator.getInstance().setShotFeasible(
            isCalculatedShotFeasible ||
            LaunchCalculator.getInstance().getShotPreset() != LaunchPreset.NONE); // shot feasible is true (if using preset)

        // Log data
        Logger.recordOutput("ShotCalculator/Shot Distance Valid?", shotDistanceValid);
        Logger.recordOutput("ShotCalculator/Drive At Goal?", driveAtGoal);
        Logger.recordOutput("ShotCalculator/Hood At Goal?", hoodAtGoal);
        Logger.recordOutput("ShotCalculator/Flywheels At Goal?", flywheelsAtGoal);

        // Clear latest shot info
        LaunchCalculator.getInstance().clearLatestShotInfo();
    }

    public void autonomousInit() {        
        autoChooser.startAuto();
    }

    public void teleopInit() {
        autoChooser.cancelAuto();
    }

    public void disabledInit() {
        if (drive.shouldCoastAfterAutoEnd()) {
            drive.coast(); // Coasts drivetrain in disabled mode if post-auto coasting is enabled
        }

        warmupExecutor.initialize();
    }

    public void disabledPeriodic() {
        autoChooser.updateAutoSelection();
        warmupExecutor.update();
    }

    public void simulationInit() {
        if (Constants.usingMapleSim) {
            for (int i = 0; i < 2; i++) { // Do twice to counteract MapleSim arena initialization effects
                drive.setPose(AllianceFlipUtil.apply(new Pose2d(1.889, 4.002, Rotation2d.kZero)));
            }
        }
    }

    public void simulationPeriodic() {
        gameViz.update();
    }

    public void test() {
        
    }
}
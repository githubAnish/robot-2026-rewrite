package org.frogforce503.robot;

import java.util.function.Consumer;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.rebuilt.maplesim.MapleSimUtil;
import org.frogforce503.lib.util.Zone;
import org.frogforce503.lib.vision.apriltagdetection.VisionMeasurement;
import org.frogforce503.robot.Constants.Mode;
import org.frogforce503.robot.auto.AutoChooser;
import org.frogforce503.robot.auto.WarmupExecutor;
import org.frogforce503.robot.commands.EjectFuelFromIntake;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.LowerClimber;
import org.frogforce503.robot.commands.RaiseClimber;
import org.frogforce503.robot.commands.ShakeIntake;
import org.frogforce503.robot.commands.ShootFuelIntoHubOrLob;
import org.frogforce503.robot.commands.drive.AimAtHubOrLob;
import org.frogforce503.robot.commands.drive.AlignToClimb;
import org.frogforce503.robot.commands.drive.TeleopDriveCommand;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.io.ClimberIO;
import org.frogforce503.robot.subsystems.climber.io.ClimberIOSim;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIO;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIOSim;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIO;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIOSim;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIO;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIOSim;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIO;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIOSim;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIO;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIOSim;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIO;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIOSim;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonSim;
import org.frogforce503.robot.subsystems.vision.io.objectdetection.ObjectDetectionIO;
import org.frogforce503.robot.subsystems.vision.io.objectdetection.ObjectDetectionIOPhotonSim;
import org.frogforce503.robot.viz.GameViz;
import org.frogforce503.robot.viz.PracticeMatchViz;
import org.frogforce503.robot.viz.VisionSimulator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    final Trigger intakeGround = driverXbox.leftTrigger();
    final Trigger ejectIntake = driverXbox.leftBumper();
    
    final Trigger shootHubOrLob = driverXbox.rightTrigger();
    final Trigger aimHubOrLob = driverXbox.rightBumper();

    final Trigger setBatterPreset = driverXbox.y();
    final Trigger setTrenchPreset = driverXbox.x();
    final Trigger setDepotPreset = driverXbox.a();

    final Trigger climb = driverXbox.b();

    // Overrides
    final Trigger toggleSlowMode = driverXbox.back();
    final Trigger toggleRobotRelative = driverXbox.start();
    final Trigger resetRobotRotation = driverXbox.povUp();
    final Trigger xWheels = driverXbox.povDown();
    final Trigger alignToClimb = driverXbox.povRight();

    // Commands
    private final TeleopDriveCommand teleopDriveCommand;

    // Other
    private final Consumer<VisionMeasurement> visionEstimateConsumer = visionMeasurement -> drive.acceptVisionMeasurement(visionMeasurement);
    
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
                    drive = new Drive(new DriveIOMapleSim());

                    intakePivot = new IntakePivot(new IntakePivotIOSim());
                    intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                    indexer = new Indexer(new IndexerIOSim());
                    feeder = new Feeder(new FeederIOSim());
                    hood = new Hood(new HoodIOSim());
                    flywheels = new Flywheels(new FlywheelsIOSim());

                    climber = new Climber(new ClimberIOSim());

                    vision =
                        new Vision(
                            visionEstimateConsumer,
                            drive::getPose,
                            new AprilTagIO[] {
                                new AprilTagIOPhotonSim(CameraName.LEFT_CAMERA, visionViz),
                                new AprilTagIOPhotonSim(CameraName.RIGHT_CAMERA, visionViz),
                                new AprilTagIOPhotonSim(CameraName.BACK_CAMERA, visionViz),
                            },
                            new ObjectDetectionIO[] {
                                new ObjectDetectionIOPhotonSim(CameraName.FUEL_CAMERA, visionViz)
                            });
                }
            }
        }

        // No-op implementations if replay or not defined above
        if (drive == null) {
            drive = new Drive(new DriveIO() {});
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

        if (vision == null) {
            vision =
                new Vision(
                    visionEstimateConsumer,
                    drive::getPose,
                    new AprilTagIO[] {},
                    new ObjectDetectionIO[] {});
        }

        // Create sim requirements
        gameViz =
            Constants.isPracticeMatch
                ? new PracticeMatchViz(drive, intakePivot, hood, flywheels, climber, visionViz)
                : new GameViz(drive, intakePivot, hood, flywheels, climber, visionViz);

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

        flywheels.setDefaultCommand(
            Commands.runOnce((() -> flywheels.setVelocity(FlywheelsConstants.IDLE)), flywheels)
                .withName("Flywheels Default Command"));

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Bind main controls
        intakeGround
            .whileTrue(new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz));

        shootHubOrLob
            .whileTrue(new ShootFuelIntoHubOrLob(drive, indexer, feeder, hood, flywheels, gameViz))
            .and(intakeGround.negate())
            .whileTrue(new ShakeIntake(intakePivot, intakeRoller).withName("ShakeIntake"));

        aimHubOrLob
            .whileTrue(new AimAtHubOrLob(drive, driverXbox));

        ejectIntake
            .whileTrue(new EjectFuelFromIntake(intakePivot, intakeRoller, indexer, feeder));

        bindShotPreset(setBatterPreset, ShotPreset.BATTER);
        bindShotPreset(setTrenchPreset, ShotPreset.TRENCH);
        bindShotPreset(setDepotPreset, ShotPreset.DEPOT);

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

    private void bindShotPreset(Trigger trigger, ShotPreset shotPreset) {
        trigger
            .onTrue(Commands.runOnce(() -> ShotCalculator.getInstance().setShotPreset(shotPreset)))
            .onFalse(Commands.runOnce(() -> ShotCalculator.getInstance().setShotPreset(ShotPreset.NONE)));
    }

    public void robotPeriodic() {
        // Calculate latest shot info
        ShotInfo shotInfo =
            ShotCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        // Check if shot feasible
        boolean shotDistanceValid = ShotCalculator.getInstance().isShotDistanceValid(drive.getPose());
        boolean driveAtGoal = MathUtil.isNear(shotInfo.driveAngle().getRadians(), drive.getPose().getRotation().getRadians(), DriveConstants.aimTolerance);
        boolean hoodAtGoal = hood.isAtAngle(shotInfo.hoodAngleRad(), HoodConstants.shootOnMoveTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(shotInfo.flywheelsVelocityRadPerSec(), FlywheelsConstants.tolerance);

        boolean isCalculatedShotFeasible =
            shotDistanceValid && driveAtGoal && hoodAtGoal && flywheelsAtGoal;

        ShotCalculator.getInstance().setShotFeasible(
            isCalculatedShotFeasible ||
            ShotCalculator.getInstance().getShotPreset() != ShotPreset.NONE); // shot feasible = true (if using preset)

        // Log data
        Logger.recordOutput("ShotCalculator/Shot Distance Valid?", shotDistanceValid);
        Logger.recordOutput("ShotCalculator/Drive At Goal?", driveAtGoal);
        Logger.recordOutput("ShotCalculator/Hood At Goal?", hoodAtGoal);
        Logger.recordOutput("ShotCalculator/Flywheels At Goal?", flywheelsAtGoal);

        // Clear latest shot info
        ShotCalculator.getInstance().clearLatestShotInfo();
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
        for (int i = 0; i < 2; i++) { // Do twice to counteract MapleSim arena initialization effects
            drive.setPose(AllianceFlipUtil.apply(new Pose2d(1.889, 4.002, Rotation2d.kZero)));
        }
    }

    public void simulationPeriodic() {
        gameViz.update();
    }

    public void test() {
        RobotModeTriggers.teleop().onTrue(Commands.run(() -> {
            MapleSimUtil.logArena(drive.getViz());

            drive.getViz().getObject("ajdoisad").setPose(FieldConstants.Tower.getPreClimbPose(drive.getPose()));
            Logger.recordOutput("ajdoisad", FieldConstants.Tower.getPreClimbPose(drive.getPose()));

            drive.getViz().getObject("ajdoisad1").setPose(FieldConstants.Tower.getClimbPose(drive.getPose()));
            Logger.recordOutput("ajdoisad1", FieldConstants.Tower.getClimbPose(drive.getPose()));

            FieldConstants.Tower.blue.log("asdausd", drive.getViz());

            new Zone(drive.getPose(), DriveConstants.bumperLength - Units.inchesToMeters(6), DriveConstants.bumperWidth - Units.inchesToMeters(6)).log("drivepose", drive.getViz());
        }));
    }
}
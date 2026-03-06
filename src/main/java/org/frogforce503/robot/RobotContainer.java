package org.frogforce503.robot;

import java.util.function.Consumer;

import org.frogforce503.lib.io.TriggerUtil;
import org.frogforce503.lib.logging.LoggedJVM;
import org.frogforce503.lib.vision.apriltagdetection.VisionMeasurement;
import org.frogforce503.robot.Constants.Mode;
import org.frogforce503.robot.auto.AutoChooser;
import org.frogforce503.robot.auto.WarmupExecutor;
import org.frogforce503.robot.commands.ClimbSequence;
import org.frogforce503.robot.commands.EjectFuelFromShooter;
import org.frogforce503.robot.commands.EjectFuelFromIntake;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.RunIndexerWhenReady;
import org.frogforce503.robot.commands.ShootFuelIntoHubOrLob;
import org.frogforce503.robot.commands.TrackTargetCommand;
import org.frogforce503.robot.commands.drive.TeleopDriveCommand;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeploy;
import org.frogforce503.robot.subsystems.climberdeploy.io.ClimberDeployIO;
import org.frogforce503.robot.subsystems.climberdeploy.io.ClimberDeployIOSim;
import org.frogforce503.robot.subsystems.climberdeploy.io.ClimberDeployIOSpark;
import org.frogforce503.robot.subsystems.climberhook.ClimberHook;
import org.frogforce503.robot.subsystems.climberhook.io.ClimberHookIO;
import org.frogforce503.robot.subsystems.climberhook.io.ClimberHookIOSim;
import org.frogforce503.robot.subsystems.climberhook.io.ClimberHookIOSpark;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
import org.frogforce503.robot.subsystems.drive.io.DriveIOPhoenix;
import org.frogforce503.robot.subsystems.leds.Leds;
import org.frogforce503.robot.subsystems.leds.LedsConstants;
import org.frogforce503.robot.subsystems.leds.io.LedsIO;
import org.frogforce503.robot.subsystems.leds.io.LedsIOCANdle;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIO;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIOSim;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIOSpark;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIO;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIOSim;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIOSpark;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIO;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIOSim;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIOSpark;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIO;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIOSim;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIOSpark;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIO;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIOSim;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIOSpark;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIO;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIOSim;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIOSpark;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIO;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIOSim;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIOSpark;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonSim;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonVision;
import org.frogforce503.robot.subsystems.vision.io.objectdetection.ObjectDetectionIO;
import org.frogforce503.robot.subsystems.vision.io.objectdetection.ObjectDetectionIOPhotonSim;
import org.frogforce503.robot.subsystems.vision.io.objectdetection.ObjectDetectionIOPhotonVision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;

/**
 * Main container for robot subsystems, commands, and controller bindings.
 * Use https://www.padcrafter.com to visualize the controller bindings.
 */
@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Vision vision;

    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;
    private Indexer indexer;
    private Feeder feeder;
    private Turret turret;
    private Hood hood;
    private Flywheels flywheels;

    private ClimberDeploy climberDeploy;
    private ClimberHook climberHook;

    private Leds leds;

    // Auto
    private final AutoChooser autoChooser;
    private final WarmupExecutor warmupExecutor;

    // Sim
    private final GameViz gameViz;
    private final VisionSimulator visionViz = new VisionSimulator();

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);

    // Main Buttons
    final Trigger intakeGround = driverXbox.leftTrigger();
    final Trigger ejectIntake = driverXbox.leftBumper(); // make this eject all

    final Trigger shootHubOrLob = driverXbox.rightTrigger();
    final Trigger ejectFlywheels = driverXbox.rightBumper(); // make this unjam

    final Trigger setBatterPreset = driverXbox.y();
    final Trigger setTrenchPreset = driverXbox.x();
    final Trigger setDepotPreset = driverXbox.a();

    final Trigger climb = driverXbox.b();

    // Overrides
    final Trigger toggleSlowMode = driverXbox.back();
    final Trigger toggleRobotRelative = driverXbox.start();
    final Trigger resetRobotRotation = driverXbox.povUp();
    final Trigger xWheels = driverXbox.povDown();

    final Trigger seedTurretRelativePosition = driverXbox.povLeft();
    final Trigger toggleAutoAssist = driverXbox.povRight();

    private final LoggedNetworkBoolean autoAssistOverride =
        new LoggedNetworkBoolean("Auto Assist Override", true); // Auto-aligning

    // Other
    private final Consumer<VisionMeasurement> visionEstimateConsumer = visionMeasurement -> drive.acceptVisionMeasurement(visionMeasurement);
    private final LoggedJVM loggedJVM = new LoggedJVM();
    
    public RobotContainer() {
        // Initialize subsystems based on robot type
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case CompBot -> {
                    drive = new Drive(new DriveIOPhoenix());
                            
                    intakePivot = new IntakePivot(new IntakePivotIOSpark());
                    intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
                    indexer = new Indexer(new IndexerIOSpark());
                    feeder = new Feeder(new FeederIOSpark());
                    turret = new Turret(new TurretIOSpark(), drive::getAngle, () -> drive.getRobotVelocity().omegaRadiansPerSecond);
                    flywheels = new Flywheels(new FlywheelsIOSpark());
                    hood = new Hood(new HoodIOSpark());

                    climberDeploy = new ClimberDeploy(new ClimberDeployIOSpark());
                    climberHook = new ClimberHook(new ClimberHookIOSpark());

                    leds = new Leds(new LedsIOCANdle());

                    vision =
                        new Vision(
                            visionEstimateConsumer,
                            drive::getPose,
                            turret::getRobotRelativeAngleRad,
                            new AprilTagIO[] {
                                new AprilTagIOPhotonVision(
                                    CameraName.TURRET_CAMERA
                                ),
                                new AprilTagIOPhotonVision(
                                    CameraName.LEFT_CAMERA
                                ),
                                new AprilTagIOPhotonVision(
                                    CameraName.RIGHT_CAMERA
                                ),
                                new AprilTagIOPhotonVision(
                                    CameraName.BACK_CAMERA
                                ),
                            },
                            new ObjectDetectionIO[] {
                                new ObjectDetectionIOPhotonVision(
                                    CameraName.FUEL_CAMERA
                                )
                            });
                }
                case PracticeBot -> {
                    
                }
                case ProgrammingBot -> {
                    
                }
                case SimBot -> {
                    drive = new Drive(new DriveIOMapleSim());

                    intakePivot = new IntakePivot(new IntakePivotIOSim());
                    intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                    indexer = new Indexer(new IndexerIOSim());
                    feeder = new Feeder(new FeederIOSim());
                    turret = new Turret(new TurretIOSim(), drive::getAngle, () -> drive.getRobotVelocity().omegaRadiansPerSecond);
                    flywheels = new Flywheels(new FlywheelsIOSim());
                    hood = new Hood(new HoodIOSim());

                    climberDeploy = new ClimberDeploy(new ClimberDeployIOSim());
                    climberHook = new ClimberHook(new ClimberHookIOSim());
                    
                    leds = new Leds(new LedsIO() {});

                    vision =
                        new Vision(
                            visionEstimateConsumer,
                            drive::getPose,
                            turret::getRobotRelativeAngleRad,
                            new AprilTagIO[] {
                                new AprilTagIOPhotonSim(
                                    CameraName.TURRET_CAMERA,
                                    visionViz
                                ),
                                new AprilTagIOPhotonSim(
                                    CameraName.LEFT_CAMERA,
                                    visionViz
                                ),
                                new AprilTagIOPhotonSim(
                                    CameraName.RIGHT_CAMERA,
                                    visionViz
                                ),
                                new AprilTagIOPhotonSim(
                                    CameraName.BACK_CAMERA,
                                    visionViz
                                ),
                            },
                            new ObjectDetectionIO[] {
                                new ObjectDetectionIOPhotonSim(
                                    CameraName.FUEL_CAMERA,
                                    visionViz
                                )
                            });
                }
            }
        }

        // No-op implementations for replay
        if (Constants.getMode() == Mode.REPLAY) {
            drive = new Drive(new DriveIO() {});

            intakePivot = new IntakePivot(new IntakePivotIO() {});
            intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
            indexer = new Indexer(new IndexerIO() {});
            feeder = new Feeder(new FeederIO() {});
            turret = new Turret(new TurretIO() {}, drive::getAngle, () -> drive.getRobotVelocity().omegaRadiansPerSecond);
            flywheels = new Flywheels(new FlywheelsIO() {});
            hood = new Hood(new HoodIO() {});

            climberDeploy = new ClimberDeploy(new ClimberDeployIO() {});
            climberHook = new ClimberHook(new ClimberHookIO() {});
            
            leds = new Leds(new LedsIO() {});

            vision =
                new Vision(
                    visionEstimateConsumer,
                    drive::getPose,
                    turret::getRobotRelativeAngleRad,
                    new AprilTagIO[] {},
                    new ObjectDetectionIO[] {});
        }

        // Create auto requirements
        autoChooser = new AutoChooser(drive, vision);
        warmupExecutor = new WarmupExecutor(drive);

        // Create sim requirements
        gameViz = new GameViz(drive, turret, hood, intakePivot, climberDeploy, visionViz);

        configureBindings();
    }

    private void configureBindings() {
        // Create commands (that have or provide dependencies elsewhere)
        final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(drive, driverXbox);
        final RunIndexerWhenReady indexerRunCommand = new RunIndexerWhenReady(indexer, intakeGround, shootHubOrLob);

        // Apply default commands
        drive.setDefaultCommand(teleopDriveCommand);

        indexer.setDefaultCommand(indexerRunCommand);
        turret.setDefaultCommand(new TrackTargetCommand(drive, vision, turret, hood)); // Requires turret and hood
        flywheels.setDefaultCommand(Commands.run(() -> flywheels.setVelocity(FlywheelsConstants.IDLE), flywheels).withName("Flywheels Idle"));

        // Bind main controls
        intakeGround.whileTrue(
            new IntakeFuelFromGround(drive, vision, intakePivot, intakeRoller, driverXbox, autoAssistOverride, gameViz.getIntakeSimulation()));

        shootHubOrLob.whileTrue(
            new ShootFuelIntoHubOrLob(
                drive, vision, feeder, turret, hood, flywheels, gameViz.getIntakeSimulation()));

        ejectIntake.whileTrue(new EjectFuelFromIntake(intakePivot, intakeRoller, indexer, feeder));
        ejectFlywheels.whileTrue(new EjectFuelFromShooter(feeder, flywheels));

        bindShotPreset(setBatterPreset, ShotPreset.BATTER);
        bindShotPreset(setTrenchPreset, ShotPreset.TRENCH);
        bindShotPreset(setDepotPreset, ShotPreset.DEPOT);

        bindClimbing(climb);

        // Bind override controls
        toggleSlowMode.onTrue(Commands.runOnce(teleopDriveCommand::toggleSlowMode));
        toggleRobotRelative.onTrue(Commands.runOnce(teleopDriveCommand::toggleRobotRelative));
        resetRobotRotation.onTrue(Commands.runOnce(drive::resetRotation));
        xWheels.onTrue(Commands.runOnce(drive::brake));

        seedTurretRelativePosition.onTrue(Commands.runOnce(turret::seedRelativePosition));
        toggleAutoAssist.onTrue(Commands.runOnce(() -> autoAssistOverride.set(!autoAssistOverride.get())));
    
        // Triggers
        Trigger shotFeasible = new Trigger(ShotCalculator.getInstance()::isFeasibleShot);

        // Rumbles controller for 0.25 sec & blinks LEDs for 1 sec once shot feasible
        shotFeasible
            .onTrue(
                Commands.parallel(
                    rumbleController(0.75, 0.25),
                    blinkLeds(LedsConstants.READY_TO_SHOOT, 1)
            ).withName("Driver Rumble Feasible Shot"));
    }

    private void bindShotPreset(Trigger trigger, ShotPreset shotPreset) {
        trigger
            .whileTrue(Commands.runOnce(() -> ShotCalculator.getInstance().setShotPreset(shotPreset)))
            .onFalse(Commands.runOnce(() -> ShotCalculator.getInstance().setShotPreset(ShotPreset.NONE)));
    }

    // Starts climb sequence and prevents other commands from interrupting it (including itself re-scheduling)
    private void bindClimbing(Trigger advanceTrigger) {
        Command climbSequence =
            new ClimbSequence(
                intakePivot, intakeRoller, indexer, feeder, turret, hood, flywheels, climberDeploy, climberHook, advanceTrigger, gameViz::setRobotHeightMeters);

        advanceTrigger.onTrue(
            climbSequence
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }

    private Command rumbleController(double value, double durationSec) {
        return
            Commands.run(() -> driverXbox.setRumble(RumbleType.kBothRumble, value))
                .withTimeout(durationSec)
                .finallyDo(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0));
    }

    private Command blinkLeds(ControlRequest pattern, double durationSec) {
        return
            Commands.sequence(
                Commands.runOnce(() -> leds.runPattern(LedsConstants.READY_TO_SHOOT)),
                Commands.waitSeconds(durationSec),
                Commands.runOnce(() -> leds.stop()));
    }

    public void robotPeriodic() {
        loggedJVM.update();

        if (RobotBase.isSimulation()) {
            gameViz.update();
        }

        // Clear shot params
        ShotCalculator.getInstance().clearShotInfo();

        Logger.recordOutput("Alliance Color", FieldConstants.getAlliance());
    }

    public void autonomousInit() {
        autoChooser.startAuto();
    }

    public void teleopInit() {
        autoChooser.close();
    }

    public void disabledInit() {
        if (drive.isCoastAfterAutoEnd()) {
            drive.coast(); // Coasts drivetrain in disabled mode if post-auto coasting is enabled
        }
        warmupExecutor.disabledInit();
    }

    public void disabledPeriodic() {
        autoChooser.periodic();
        warmupExecutor.disabledPeriodic();
        turret.seedRelativePosition();
    }

    public void test() {
        // Schedule the TuneShot command (helps tune shotmaps) by uncommenting the following 2 lines
        // RobotModeTriggers.teleop().onTrue(
        //     new TuneShot(drive, turret, hood, flywheels, gameViz.getIntakeSimulation(), true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }
}
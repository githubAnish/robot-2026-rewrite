package org.frogforce503.robot;

import java.util.function.Consumer;

import org.frogforce503.lib.io.TriggerUtil;
import org.frogforce503.lib.logging.LoggedJVM;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;
import org.frogforce503.robot.auto.AutoChooser;
import org.frogforce503.robot.auto.WarmupExecutor;
import org.frogforce503.robot.commands.ClimbSequence;
import org.frogforce503.robot.commands.EjectFuelFromFlywheels;
import org.frogforce503.robot.commands.EjectFuelFromIntake;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.IntakeFuelFromOutpost;
import org.frogforce503.robot.commands.LobFuelIntoAlliance;
import org.frogforce503.robot.commands.PrepForLobFuelIntoAlliance;
import org.frogforce503.robot.commands.PrepForShootFuelIntoHub;
import org.frogforce503.robot.commands.ShootFuelIntoHub;
import org.frogforce503.robot.commands.drive.TeleopDriveCommand;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.io.ClimberIO;
import org.frogforce503.robot.subsystems.climber.io.ClimberIOSim;
import org.frogforce503.robot.subsystems.climber.io.ClimberIOSpark;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
import org.frogforce503.robot.subsystems.drive.io.DriveIOPhoenix;
import org.frogforce503.robot.subsystems.leds.Leds;
import org.frogforce503.robot.subsystems.leds.LedsRequest;
import org.frogforce503.robot.subsystems.leds.io.LedsIO;
import org.frogforce503.robot.subsystems.leds.io.LedsIOCANdle;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIO;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIOSim;
import org.frogforce503.robot.subsystems.superstructure.feeder.io.FeederIOSpark;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
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
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.frogforce503.robot.subsystems.vision.apriltag_detection.AprilTagIO;
import org.frogforce503.robot.subsystems.vision.object_detection.ObjectDetectionIO;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Vision vision;
    private final Superstructure superstructure;
    private Climber climber;
    private Leds leds;

    // Auto
    private final AutoChooser autoChooser;
    private final WarmupExecutor warmupExecutor;

    // Sim
    private final GameViz gameViz;
    private final VisionSimulator visionViz = new VisionSimulator();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Trigger driverLeftPaddle = driver.leftPaddle();
    private final Trigger driverRightPaddle = driver.rightPaddle();

    // Overrides
    private final LoggedNetworkBoolean autoAssistEnabled = // Includes auto-aligning and auto-aiming
        new LoggedNetworkBoolean("Auto Assist Enabled", true);

    // Other
    private final Consumer<VisionMeasurement> visionEstimateConsumer =
        visionMeasurement ->
            drive.acceptVisionMeasurement(visionMeasurement);

    private final LoggedJVM loggedJVM = new LoggedJVM();
    
    public RobotContainer() {
        IntakePivot intakePivot = null;
        IntakeRoller intakeRoller = null;
        Indexer indexer = null;
        Feeder feeder = null;
        Turret turret = null;
        Flywheels flywheels = null;
        Hood hood = null;

        // Initialize subsystems based on robot type
        switch (Constants.getRobot()) {
            case CompBot -> {
                drive = new Drive(new DriveIOPhoenix());
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getPose,
                        new AprilTagIO[] {},
                        new ObjectDetectionIO[] {});

                intakePivot = new IntakePivot(new IntakePivotIOSpark());
                intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
                indexer = new Indexer(new IndexerIOSpark());
                feeder = new Feeder(new FeederIOSpark());
                turret = new Turret(new TurretIOSpark());
                flywheels = new Flywheels(new FlywheelsIOSpark());
                hood = new Hood(new HoodIOSpark());

                climber = new Climber(new ClimberIOSpark());

                leds = new Leds(new LedsIOCANdle());
            }
            case PracticeBot -> {
                drive = new Drive(new DriveIOPhoenix());
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getPose,
                        new AprilTagIO[] {},
                        new ObjectDetectionIO[] {});

                intakePivot = new IntakePivot(new IntakePivotIOSpark());
                intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
                indexer = new Indexer(new IndexerIOSpark());
                feeder = new Feeder(new FeederIOSpark());
                turret = new Turret(new TurretIOSpark());
                flywheels = new Flywheels(new FlywheelsIOSpark());
                hood = new Hood(new HoodIOSpark());

                climber = new Climber(new ClimberIOSpark());
                
                leds = new Leds(new LedsIOCANdle());
            }
            case SimBot -> {
                drive = new Drive(new DriveIOMapleSim());
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getPose,
                        new AprilTagIO[] {},
                        new ObjectDetectionIO[] {});

                intakePivot = new IntakePivot(new IntakePivotIOSim());
                intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                indexer = new Indexer(new IndexerIOSim());
                feeder = new Feeder(new FeederIOSim());
                turret = new Turret(new TurretIOSim());
                flywheels = new Flywheels(new FlywheelsIOSim());
                hood = new Hood(new HoodIOSim());

                climber = new Climber(new ClimberIOSim());
                
                leds = new Leds(new LedsIO() {});
            }
            case ProgrammingBot -> {
                drive = new Drive(new DriveIOPhoenix());
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getPose,
                        new AprilTagIO[] {},
                        new ObjectDetectionIO[] {});

                intakePivot = new IntakePivot(new IntakePivotIO() {});
                intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
                indexer = new Indexer(new IndexerIO() {});
                feeder = new Feeder(new FeederIO() {});
                turret = new Turret(new TurretIO() {});
                flywheels = new Flywheels(new FlywheelsIO() {});
                hood = new Hood(new HoodIO() {});

                climber = new Climber(new ClimberIO() {});
                
                leds = new Leds(new LedsIO() {});
            }
            default -> {
                System.err.println("What happened here?" + ErrorUtil.attachJavaClassName(RobotContainer.class));
            }
        }

        // Create virtual subsystems
        superstructure =
            new Superstructure(
                intakePivot,
                intakeRoller,
                indexer,
                feeder,
                turret,
                flywheels,
                hood,
                drive::getPose);

        // Create auto requirements
        autoChooser = new AutoChooser(drive, vision, superstructure);
        warmupExecutor = new WarmupExecutor(drive, autoChooser);

        // Create sim requirements
        gameViz = new GameViz(drive, visionViz);

        configureButtonBindings();
        configureTriggers();
    }

    private void configureButtonBindings() {
        // Main controls
        drive.setDefaultCommand(new TeleopDriveCommand(drive, driver));

        driver.leftTrigger().whileTrue(new IntakeFuelFromGround(drive, vision, superstructure, driver, autoAssistEnabled::get));
        driver.leftBumper().whileTrue(new IntakeFuelFromOutpost(drive, vision, superstructure, autoAssistEnabled::get));

        driver.rightTrigger().whileTrue(new ShootFuelIntoHub(drive, vision, superstructure, autoAssistEnabled::get)); // auto aim + shoot, see past examples
        driver.rightBumper().whileTrue(new LobFuelIntoAlliance(drive, vision, superstructure, autoAssistEnabled::get));
        
        driverLeftPaddle.whileTrue(new EjectFuelFromIntake(superstructure));
        driverRightPaddle.whileTrue(new EjectFuelFromFlywheels(superstructure));

        bindPresets(driver.y(), ShotPreset.BATTER);
        bindPresets(driver.b(), ShotPreset.DEPOT);
        bindPresets(driver.a(), ShotPreset.OUTPOST);
        bindPresets(driver.x(), ShotPreset.TRENCH);

        bindClimbing(driver.povUp(), true);

        // Overrides
        driver.back().onTrue(Commands.runOnce(drive::toggleSlowMode));
        driver.start().onTrue(Commands.runOnce(drive::toggleRobotRelative));
        operator.povUp().onTrue(Commands.runOnce(drive::resetRotation));

        operator.a().onTrue(Commands.runOnce(() -> drive.brake()));
    }

    private void configureTriggers() {
        Trigger inAllianceZone =
            new Trigger(() ->
                FieldInfo.isRed()
                    ? drive.getPose().getX() > FieldConstants.Lines.redInitLineX
                    : drive.getPose().getX() < FieldConstants.Lines.blueInitLineX
            );

        inAllianceZone
            .onTrue(new PrepForShootFuelIntoHub(drive, vision, superstructure))
            .onFalse(new PrepForLobFuelIntoAlliance(drive, vision, superstructure));

        Trigger camerasConnected = new Trigger(() -> true); // TODO: Make a method for this in Vision.java

        // If cameras disconnected for 5 seconds, then leds will blink red until cameras re-connected
        camerasConnected
            .debounce(5.0, DebounceType.kBoth)
            .onTrue(
                Commands.runOnce(
                    () -> leds.setCameraDisconnected(false))
                        .ignoringDisable(true))
            .onFalse(
                Commands.runOnce(
                    () -> leds.setCameraDisconnected(true))
                        .ignoringDisable(true));

        Trigger shotFeasible = new Trigger(superstructure::isFeasibleShot);

        // If shot is feasible and driver knows they can take shot, then rumbles driver for 0.25 sec and blinks LEDs
        // Rumbling logic can change in the future
        shotFeasible
            .onTrue(
                Commands.parallel(
                    setDriverRumble(0.75, 0.25),
                    Commands.runOnce(() -> leds.runPattern(LedsRequest.READY_TO_SHOOT))
            ));
    }

    private Command setDriverRumble(double value, double durationSec) {
        return
            Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, value))
                .withTimeout(durationSec)
                .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0))
                .withName("setDriverRumble");
    }

    private void bindPresets(Trigger trigger, ShotPreset shotPreset) {
        trigger
            .onTrue(Commands.runOnce(() -> superstructure.setShotPreset(shotPreset)))
            .onFalse(Commands.runOnce(() -> superstructure.setShotPreset(ShotPreset.NONE)));
    }

    // Cancel incoming commands to ensure no unintended / undesirable behaior occurs outside of the climb sequence
    private void bindClimbing(Trigger advanceTrigger, boolean cancelIncomingCommands) {
        Command climbSequence = new ClimbSequence(superstructure, climber, advanceTrigger);

        advanceTrigger.onTrue(
            cancelIncomingCommands
                ? climbSequence.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                : climbSequence);
    }

    public void robotPeriodic() {
        loggedJVM.update();

        if (RobotBase.isSimulation()) {
            gameViz.update();
        }

        Logger.recordOutput("Alliance Color", FieldInfo.getAlliance());
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
        
        superstructure.seedTurretRelativePosition();
    }

    public void test() {
        
    }
}
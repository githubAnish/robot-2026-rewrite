package org.frogforce503.robot;

import java.util.Map;
import java.util.function.Consumer;

import org.frogforce503.lib.io.DoublePressTracker;
import org.frogforce503.lib.io.TriggerUtil;
import org.frogforce503.lib.logging.LoggedJVM;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FFSelectCommand;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;
import org.frogforce503.robot.auto.AutoChooser;
import org.frogforce503.robot.auto.WarmupExecutor;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.IntakeFuelFromOutpost;
import org.frogforce503.robot.commands.ShootFuelIntoHub;
import org.frogforce503.robot.commands.drive.TeleopDriveCommand;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveIOMapleSim;
import org.frogforce503.robot.subsystems.drive.DriveIOPhoenix;
import org.frogforce503.robot.subsystems.leds.Leds;
import org.frogforce503.robot.subsystems.leds.LedsIO;
import org.frogforce503.robot.subsystems.leds.LedsIOCANdle;
import org.frogforce503.robot.subsystems.leds.LedsRequest;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.SuperstructureMode;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class})
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Vision vision;
    private final Superstructure superstructure;
    private Leds leds;

    // Auto
    private final AutoChooser autoChooser;

    // Sim
    private final GameViz gameViz;
    private final VisionSimulator visionViz = new VisionSimulator();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Trigger driverLeftPaddle = driver.leftPaddle();
    private final Trigger driverRightPaddle = driver.rightPaddle();

    // Other
    private final WarmupExecutor warmupExecutor;

    private final Consumer<VisionMeasurement> visionEstimateConsumer =
        visionMeasurement ->
            drive.acceptVisionMeasurement(visionMeasurement);

    private final LoggedJVM loggedJVM = new LoggedJVM();

    // Overrides
    private final LoggedNetworkBoolean autoDrivingEnabled =
        new LoggedNetworkBoolean("Auto Driving Enabled", true);
    
    public RobotContainer() {
        // Define subsystems

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
                leds = new Leds(new LedsIO() {});
            }
            default -> {
                System.err.println("What happened here?" + ErrorUtil.attachJavaClassName(RobotContainer.class));
            }
        }

        // Create virtual subsystems
        superstructure =
            new Superstructure(drive::getPose);

        // Create auto requirements
        autoChooser = new AutoChooser(drive, superstructure);
        warmupExecutor = new WarmupExecutor(drive, autoChooser);

        // Create sim requirements
        gameViz = new GameViz(drive, visionViz);

        configureButtonBindings();
        configureTriggers();
        configureOther();
    }

    private void configureButtonBindings() {
        // Main controls
        driver.leftTrigger().whileTrue(
            new FFSelectCommand<>(
                Map.of(
                    SuperstructureMode.FUEL_GROUND, new IntakeFuelFromGround(),
                    SuperstructureMode.FUEL_OUTPOST, new IntakeFuelFromOutpost()
                ),
                superstructure::getCurrentMode));

        driver.rightTrigger().whileTrue(new ShootFuelIntoHub()); // auto aim + shoot, see past examples

        driver.a().onTrue(Commands.runOnce(() -> superstructure.setCurrentMode(SuperstructureMode.FUEL_GROUND)));
        driver.b().onTrue(Commands.runOnce(() -> superstructure.setCurrentMode(SuperstructureMode.FUEL_OUTPOST)));
        
        // shoot from flywheel
        // intake from ground (+fetch), intake from outpost,
        // eject from intake, eject from shooter
        // batter shot & score from depot & outpost & trench & bump (maybe some more presets)

        // Overrides
        driver.back().onTrue(Commands.runOnce(drive::toggleSlowMode));
        driver.start().onTrue(Commands.runOnce(drive::toggleRobotRelative));
        operator.povUp().onTrue(Commands.runOnce(drive::resetRotation));
    }

    private void configureTriggers() {
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

        Trigger gotPiece = new Trigger(() -> true);

        // If piece got, rumbles driver for 0.25 sec and blinks LEDs
        gotPiece
            .onTrue(
                Commands.parallel(
                    setDriverRumble(0.75, 0.25),
                    Commands.runOnce(() -> leds.runPattern(LedsRequest.GOT_PIECE))
            ));
    }

    private void configureOther() {
        // Set default commands
        drive.setDefaultCommand(new TeleopDriveCommand(drive, driver));
    }

    private Command setDriverRumble(double value, double duration) {
        return
            Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, value))
                .withTimeout(duration)
                .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0))
                .withName("setDriverRumble");
    }

    public void robotPeriodic() {
        if (RobotBase.isSimulation()) {
            gameViz.update();
        }

        loggedJVM.update();

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
    }

    public void test() {
        
    }
}
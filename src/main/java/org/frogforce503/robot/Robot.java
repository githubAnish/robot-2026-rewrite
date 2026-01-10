// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.reflect.Field;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.lib.util.NTClientLogger;
import org.frogforce503.robot.constants.hardware.RobotHardware;
import org.frogforce503.robot.constants.hardware.RobotHardwareCompBot;
import org.frogforce503.robot.constants.hardware.RobotHardwarePracticeBot;
import org.frogforce503.robot.constants.hardware.RobotHardwareProgrammingBot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private final double loopOverrunWarningTimeout = 0.2;

  public static RobotHardware bot;
  private RobotContainer robotContainer;
  
  /*
   * Robot Constructor 
   */
  public Robot() {
    bot =
      switch (Constants.getRobot()) {
        case CompBot, SimBot -> new RobotHardwareCompBot();
        case PracticeBot -> new RobotHardwarePracticeBot();
        case ProgrammingBot -> new RobotHardwareProgrammingBot();
      };
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "FF2026_" + Constants.getRobot().name().toUpperCase()); // Set a metadata value

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Disable Unnecessary logging
    SignalLogger.enableAutoLogging(false);

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }

    // Disable alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Initialize RobotContainer
    robotContainer = new RobotContainer();
    robotContainer.test();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    
    // Run virtual subsystems
    VirtualSubsystem.periodicAll();

    // Run command scheduler
    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    SmartDashboard.putData(CommandScheduler.getInstance());

    // Log NT client list
    NTClientLogger.log();

    robotContainer.robotPeriodic();

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  @Override
  public void autonomousInit() {
    robotContainer.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
package org.frogforce503.robot;

import java.lang.reflect.Field;

import org.frogforce503.lib.logging.LoggedJVM;
import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.logging.NTClientLogger;
import org.frogforce503.lib.rebuilt.sim.maplesim.MapleSimUtil;
import org.frogforce503.lib.rebuilt.sim.HubShiftUtil;
import org.frogforce503.lib.util.Elastic;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private final double loopPeriodWatchdogSecs = 0.2;

    private final LoggedJVM loggedJVM = new LoggedJVM();
    private final RobotContainer robotContainer;
    
    public Robot() {
        Logger.recordMetadata("ProjectName", "FF2026_" + Constants.getRobot().name().toUpperCase()); // Set a metadata value

        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL:
                // Running on real robot, log to USB stick
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible

                String inPath = LogFileUtil.findReplayLog();
                String outPath = LogFileUtil.addPathSuffix(inPath, "_sim");

                Logger.setReplaySource(new WPILOGReader(inPath));
                Logger.addDataReceiver(new WPILOGWriter(outPath));
                break;
        }

        // Disable unnecessary logging
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();
        StatusLogger.disableAutoLogging();

        // Start AdvantageKit logger
        Logger.start();

        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopPeriodWatchdogSecs);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopPeriodWatchdogSecs);

        // Configure brownout voltage
        RobotController.setBrownoutVoltage(6.0);

        // Configure sim
        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true); // Silence joystick warnings in sim

            RoboRioSim.setTeamNumber(503);
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.notifyNewData();

            MapleSimUtil.initializeArena(); // Setup MapleSim arena
        }

        // Initialize RobotContainer
        robotContainer = new RobotContainer();
        robotContainer.test();
    }

    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();

        // Run command scheduler
        CommandScheduler.getInstance().run();
        SmartDashboard.putData(CommandScheduler.getInstance());
        LoggedTracer.record("CommandScheduler");

        // Log NT client list
        NTClientLogger.log();

        // Log JVM info
        loggedJVM.update();

        // Put alliance color on dashboard
        Logger.recordOutput("Alliance Color", FieldConstants.getAlliance());
        
        robotContainer.robotPeriodic();

        // Record cycle time
        LoggedTracer.record("RobotPeriodic");
    }

    @Override
    public void autonomousInit() {
        if (RobotBase.isSimulation() && Constants.isPracticeMatch) {
            HubShiftUtil.initialize();
        }

        robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (RobotBase.isSimulation() && Constants.isPracticeMatch) {
            HubShiftUtil.initialize();
        }

        robotContainer.teleopInit();

        // Select Teleop Tab on Elastic
        Elastic.selectTab("Teleop");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        if (RobotBase.isSimulation() && Constants.isPracticeMatch) {
            HubShiftUtil.initialize();
        }

        robotContainer.disabledInit();

        // Select Autonomous Tab on Elastic
        Elastic.selectTab("Autonomous");
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.disabledPeriodic();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto(); // Reset MapleSim field

        robotContainer.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.simulationPeriodic();
    }
}
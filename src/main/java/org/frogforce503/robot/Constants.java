package org.frogforce503.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;

/** Class containing global configuration variables describing current robot and runtime mode. */
public class Constants {
    @Getter private static RobotType robot = RobotType.SimBot;

    public static final boolean usingMapleSim = true;
    public static final boolean isPracticeMatch = true;

    public static final double loopPeriodSecs = LoggedRobot.defaultPeriodSecs;

    public static Mode getMode() {
        return switch (robot) {
            case ProgrammingBot, PracticeBot, CompBot -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SimBot -> Mode.SIM;
        };
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        CompBot, PracticeBot, ProgrammingBot, SimBot
    }
}
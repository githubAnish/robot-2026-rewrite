package org.frogforce503.robot;

import org.frogforce503.robot.constants.field.FieldVenue;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;

/** Class containing global configuration variables describing current robot, runtime mode, & field venue. */
public final class Constants {
  @Getter private static RobotType robot = RobotType.SimBot;
  public static final FieldVenue fieldVenue = FieldVenue.Shop;

  public static final double loopPeriodSecs = LoggedRobot.defaultPeriodSecs;
  public static final double loopPeriodWatchdogSecs = 0.2;
  public static final boolean useAllianceFlipping = false;

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
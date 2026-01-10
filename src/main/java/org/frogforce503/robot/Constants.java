package org.frogforce503.robot;

import org.frogforce503.robot.constants.field.FieldVenue;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/** This class contains global configuration variables describing the current robot, runtime mode, and field venue. */
public final class Constants {
  public static final double loopPeriodSecs = LoggedRobot.defaultPeriodSecs;
  public static final boolean useAllianceFlipping = false;

  private static RobotType robotType = RobotType.SimBot;
  public static final FieldVenue fieldVenue = FieldVenue.Shop;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SimBot) {
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError).set(true);
        robotType = RobotType.CompBot;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case ProgrammingBot, PracticeBot, CompBot -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SimBot -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    CompBot, PracticeBot, ProgrammingBot, SimBot
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SimBot) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotType.CompBot) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
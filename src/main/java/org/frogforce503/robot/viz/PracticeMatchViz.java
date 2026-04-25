package org.frogforce503.robot.viz;

import org.frogforce503.lib.rebuilt.sim.HubShiftUtil;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.launcher.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.launcher.hood.Hood;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;

/** Adds practice match elements, including score tracking and Hub shifts, to GameViz. */
public class PracticeMatchViz extends GameViz {
    private int score = 0;
    
    public PracticeMatchViz(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        VisionSimulator visionViz
    ) {
        super(drive, intakePivot, intakeRoller, hood, flywheels, climber, visionViz);
    }

    @Override
    public void update() {
        super.update();
        
        // Log score
        Logger.recordOutput("PracticeMatchViz/Score", score);

        // Log hub shifts
        Logger.recordOutput(
            "PracticeMatchViz/Remaining Shift Time",
            String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));

        Logger.recordOutput(
            "PracticeMatchViz/Shift Active?",
            HubShiftUtil.getShiftedShiftInfo().active());

        Logger.recordOutput(
            "PracticeMatchViz/Current Shift",
            HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    }

    @Override
    public void startIntake() {
        if (isMatchEnded()) {
            return;
        }
        
        super.startIntake();
    }

    @Override
    public void shootFuel() {
        if (isMatchEnded()) {
            return;
        }

        super.shootFuel(
            () -> {
                if (HubShiftUtil.getShiftedShiftInfo().active() && !isMatchEnded()) {
                    score++;
                }
            });
    }

    public void stopClimb() {
        super.stopClimb();

        if (climbSim.hasClimbed() && !isMatchEnded()) {
            score += RobotState.isAutonomous() ? 15 : 10;
        }
    }

    private boolean isMatchEnded() {
        return HubShiftUtil.getShiftedShiftInfo().remainingTime() <= 0;
    }
}
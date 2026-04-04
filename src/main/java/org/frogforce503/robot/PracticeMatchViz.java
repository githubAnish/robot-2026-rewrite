package org.frogforce503.robot;

import org.frogforce503.lib.rebuilt.HubShiftUtil;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.littletonrobotics.junction.Logger;

/** Simulates the field, including interaction with & movement of game elements. Uses physics simulation. */
public class PracticeMatchViz extends GameViz {
    private int fuelScoredInMatch = 0;
    
    public PracticeMatchViz(
        Drive drive,
        IntakePivot intakePivot,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        VisionSimulator visionViz
    ) {
        super(drive, intakePivot, hood, flywheels, climber, visionViz);
    }

    public void update() {
        super.update();
        
        // Log score
        Logger.recordOutput("PracticeMatchViz/Score", fuelScoredInMatch);

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

    public void shootFuel(boolean needFuelFromIntakeForShoot) {
        boolean matchEnded = HubShiftUtil.getShiftedShiftInfo().remainingTime() <= 0;

        if (matchEnded) {
            return;
        }

        super.shootFuel(
            needFuelFromIntakeForShoot,
            () -> {
                if (HubShiftUtil.getShiftedShiftInfo().active() && !matchEnded) {
                    fuelScoredInMatch++;
                }
            });
    }
}
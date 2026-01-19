package org.frogforce503.robot.auto;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.lib.auto.AutoFactoryConfigurator;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.rebuilt.ProximityUtil;
import org.frogforce503.robot.FieldInfo;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class AutoChooser {
    // Requirements
    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;
    private final AutoFactory choreoAutoFactory;

    // Dashboard
    @Getter private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    // State
    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    public AutoChooser(Drive drive, Vision vision, Superstructure superstructure) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;

        this.choreoAutoFactory = AutoFactoryConfigurator.configureChoreo(drive);
        AutoFactoryConfigurator.configurePathPlanner(drive);

        configureAutos();

        if (Robot.isSimulation()) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    private void configureAutos() {
        // Random test auto
        routineChooser.addDefaultOption(
            "Test",
            new AutoMode(drive, vision, superstructure) {

                @Override
                public Command getCommand() {
                    return Commands.runOnce(() -> drive.setPose(GeomUtil.toPose2d(new Translation2d(5,5))));
                }

                @Override
                public List<Pose2d> getPoses() {
                    return new ArrayList<>(List.of(Pose2d.kZero));
                }
            
        });

        // routineChooser.addDefaultOption("Drive Straight Test", new DriveStraightTest(drive, vision, superstructure));
    }

    private void logTrajectory(Pose2d... trajectory) {
        FieldInfo.getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }

    // Public methods
    public void startAuto() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            return;
        }

        autoCommand = selectedAuto.getCommand();

        if (autoCommand != null) {
            autoCommand.schedule();   
        }
    }

    public void periodic() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            logTrajectory(); // Clear poses

        } else if (selectedAuto != lastSelectedAuto) {
            List<Pose2d> trajectoryPoses = selectedAuto.getPoses();
            Pose2d start = trajectoryPoses.get(0);

            logTrajectory(trajectoryPoses.toArray(Pose2d[]::new));
        
            // Reset pose if drive close to trajectory start
            if (ProximityUtil.getDistanceFromPose(drive, start) <= Units.inchesToMeters(6)) {
                drive.setPose(start);
            }
        }

        lastSelectedAuto = selectedAuto;
    }

    public void close() {
        logTrajectory(); // Clear poses

        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }
}
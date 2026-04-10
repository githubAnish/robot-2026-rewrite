package org.frogforce503.robot.viz;

import org.frogforce503.lib.rebuilt.BumpPhysicsSim;
import org.frogforce503.lib.rebuilt.ClimbPhysicsSim;
import org.frogforce503.lib.rebuilt.FuelVisualizer;
import org.frogforce503.lib.rebuilt.maplesim.MapleSimUtil;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class GameViz {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final Hood hood;
    private final Flywheels flywheels;

    private final VisionSimulator visionViz;
    private final SuperstructureViz superstructureViz = new SuperstructureViz();
    private final BumpPhysicsSim bumpSim;
    private final ClimbPhysicsSim climbSim;

    private IntakeSimulation intakeSimulation;

    // Shoot Sim Constants
    private final Timer shotTimer = new Timer();

    public GameViz(
        Drive drive,
        IntakePivot intakePivot,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        VisionSimulator visionViz
    ) {
        this.drive = drive;
        this.intakePivot = intakePivot;
        this.hood = hood;
        this.flywheels = flywheels;
        this.visionViz = visionViz;

        this.bumpSim = new BumpPhysicsSim(drive);
        this.climbSim = new ClimbPhysicsSim(drive, climber);  // ADD

        if (RobotBase.isSimulation()) {
            intakeSimulation = MapleSimUtil.createIntake(drive.getMapleSimDrive().mapleSimDrive);
            
            for (int i = 0; i < 8; i++) {
                intakeSimulation.addGamePieceToIntake();
            }
        }
    }

    public void update() {
        // Apply bump physics
        Pose3d drivePose3d = bumpSim.update();

        // Apply climb physics
        drivePose3d = climbSim.update(drivePose3d);

        // Update visualizers
        visionViz.update(drive.getPose());
        superstructureViz.update(drivePose3d, hood.getAngleRad(), intakePivot.getAngleRad());

        // Visualize fuel
        Translation3d[] fuelInHopper =
            FuelVisualizer.visualizeFuelInHopper(drivePose3d, intakeSimulation.getGamePiecesAmount());

        Translation3d[] fuelTranslations =
            FuelVisualizer.visualizeFuelInField();

        // Log data
        Logger.recordOutput("GameViz/DrivePose3d", drivePose3d);
        Logger.recordOutput("GameViz/FuelTranslations", fuelTranslations);
        Logger.recordOutput("GameViz/NumFuelInRobot", intakeSimulation.getGamePiecesAmount());
        Logger.recordOutput("GameViz/FuelInHopper", fuelInHopper);
    }

    public void startIntake() {
        intakeSimulation.startIntake();
    }

    public void stopIntake() {
        intakeSimulation.stopIntake();
    }

    public void shootFuel(boolean needFuelFromIntakeForShoot, Runnable onScore) {
        MapleSimUtil.shootFuel(
            drive.getPose(),
            drive.getFieldVelocity(),
            hood.getAngleRad(),
            flywheels.getVelocityRadPerSec(),
            intakeSimulation,
            shotTimer,
            needFuelFromIntakeForShoot,
            onScore);
    }

    public void shootFuel(boolean needFuelFromIntakeForShoot) {
        shootFuel(needFuelFromIntakeForShoot, () -> {});
    }

    public void startClimb() {
        climbSim.startClimb();
    }

    public void climb() {
        climbSim.climb();
    }

    public void stopClimb() {
        climbSim.stopClimb();
    }
}
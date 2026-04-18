package org.frogforce503.robot.viz;

import org.frogforce503.lib.rebuilt.sim.maplesim.MapleSimUtil;
import org.frogforce503.lib.rebuilt.sim.BumpPhysicsSim;
import org.frogforce503.lib.rebuilt.sim.ClimbPhysicsSim;
import org.frogforce503.lib.rebuilt.sim.FuelShotQuantityCalculator;
import org.frogforce503.lib.rebuilt.sim.FuelViz;
import org.frogforce503.lib.rebuilt.sim.TrenchCollisionSim;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class GameViz {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Hood hood;
    private final Flywheels flywheels;
    private final Climber climber;

    private final VisionSimulator visionViz;
    private final SuperstructureViz superstructureViz = new SuperstructureViz();
    private final BumpPhysicsSim bumpSim;
    @Getter private final TrenchCollisionSim trenchCollisionSim;
    protected final ClimbPhysicsSim climbSim;

    private IntakeSimulation intakeSimulation;

    // Shoot Constants
    private final double leftMostFuelPositionOffset = Units.inchesToMeters(-8);
    private final double rightMostFuelPositionOffset = Units.inchesToMeters(10);
    private final double shooterFireRateBallsPerSec = 7; // How many balls can shooter fire within 1 sec?
    private final Timer shotTimer = new Timer();

    public GameViz(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        VisionSimulator visionViz
    ) {
        this.drive = drive;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.hood = hood;
        this.flywheels = flywheels;
        this.climber = climber;

        this.visionViz = visionViz;
        this.bumpSim = new BumpPhysicsSim(drive);
        this.trenchCollisionSim = new TrenchCollisionSim(superstructureViz);
        this.climbSim = new ClimbPhysicsSim(drive, climber);

        if (RobotBase.isSimulation() && Constants.usingMapleSim) {
            intakeSimulation = MapleSimUtil.createIntake(drive.getMapleSimDrive().mapleSimDrive);
            
            // Fill preload fuel
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

        // Check fuel in robot
        int fuelInRobot = getFuelInRobot();

        // Update visualizers
        visionViz.update(drive.getPose());
        superstructureViz.update(drivePose3d, hood.getAngleRad(), intakePivot.getAngleRad(), climber.getHeightMeters(), fuelInRobot);

        // Visualize fuel
        Translation3d[] fuelInHopper =
            FuelViz.visualizeFuelInHopper(
                drivePose3d,
                fuelInRobot,
                superstructureViz.getCurrentLinearExtensionX(),
                superstructureViz.getCurrentVerticalLift(),
                superstructureViz.getCurrentDiagonalAngleRad(),
                intakeRoller.getVelocityRadPerSec() > 1e-9);

        // Log data
        Logger.recordOutput("GameViz/DrivePose3d", drivePose3d);
        Logger.recordOutput("GameViz/FuelTranslations", FuelViz.visualizeFuelInField());
        Logger.recordOutput("GameViz/NumFuelInRobot", fuelInRobot);
        Logger.recordOutput("GameViz/FuelInHopper", fuelInHopper);
    }

    public int getFuelInRobot() {
        return Constants.usingMapleSim ? intakeSimulation.getGamePiecesAmount() : 0;
    }

    public void startIntake() {
        intakeSimulation.startIntake();
    }

    public void stopIntake() {
        intakeSimulation.stopIntake();
    }

    public void shootFuel(Runnable onScore) {
        int available = intakeSimulation.getGamePiecesAmount();

        if (available <= 0) {
            return; // Don't shoot balls if there are none
        }

        double shotDelaySec = 1.0 / shooterFireRateBallsPerSec;

        // Allow very first shot (timer not used yet, get() == 0.0), or when cooldown has elapsed
        if (shotTimer.isRunning() && !shotTimer.hasElapsed(shotDelaySec)) {
            return; // Cooldown not done; skip creating new projectile
        }

        // Check fuel amount
        int fuelToShoot = FuelShotQuantityCalculator.computeFuelToShoot(available);

        // Index fuel
        for (int i = 0; i < fuelToShoot; i++) {
            intakeSimulation.obtainGamePieceFromIntake();
        }

        // Shoot fuel
        double step = (fuelToShoot > 1) ? (rightMostFuelPositionOffset - leftMostFuelPositionOffset) / (fuelToShoot - 1) : 0.0;

        for (int i = 0; i < fuelToShoot; i++) {
            double offset = (fuelToShoot == 1) ? 0.0 : leftMostFuelPositionOffset + i * step;

            MapleSimUtil.createFuelProjectile(
                drive.getPose(),
                drive.getFieldVelocity(),
                hood.getAngleRad(),
                flywheels.getVelocityRadPerSec(),
                new Transform2d(0.0, offset, Rotation2d.kZero),
                onScore);
        }

        // Restart cooldown timer after firing
        shotTimer.restart();
    }

    public void shootFuel() {
        shootFuel(() -> {});
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
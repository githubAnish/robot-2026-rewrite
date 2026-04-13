package org.frogforce503.lib.rebuilt;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.lib.rebuilt.maplesim.MapleSimUtil;
import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FuelViz {
    private static final double D = Units.inchesToMeters(5.91); 
    // Reduced from 0.78 to 0.72 to "clump" balls tighter and fit more per row
    private static final double S = D * 0.72;                    

    private static final double START_X = -0.05; 
    private static final double FLOOR_Z  = 0.325;

    // Increased WIDTH to 5 to fill the gaps on the sides
    private static final int DEPTH  = 6;  
    private static final int WIDTH  = 5; 
    private static final int HEIGHT = 4;

    // Adjusted heights to maintain the staircase profile with more balls
    // This profile provides 18 slots per "slice"
    private static final int[] ROW_HEIGHTS = {4, 4, 4, 3, 2, 1};

    private static final Translation3d[] SLOTS = buildSlots();

    private static Translation3d[] buildSlots() {
        List<Translation3d> slots = new ArrayList<>();

        // 1. Fill by LAYER first (Gravity)
        for (int layer = 0; layer < HEIGHT; layer++) {
            double z = FLOOR_Z + layer * S;
            
            // 2. Fill by ROW (Shooter -X to Intake +X)
            for (int row = 0; row < DEPTH; row++) {
                // Keep the staircase profile
                if (layer >= ROW_HEIGHTS[row]) continue;

                // Keep your shift logic for upper layers
                double xOffset = (layer > 0) ? 0.08 : 0.0;
                double x = START_X + S / 2.0 + row * S + xOffset;
                
                // 3. Fill side-to-side
                for (int col = 0; col < WIDTH; col++) {
                    double y = (col - (WIDTH - 1) / 2.0) * S;
                    slots.add(new Translation3d(x, y, z));
                }
            }
        }
        return slots.toArray(new Translation3d[0]);
    }

    private FuelViz() {}

    // Inside FuelViz.java
    private static final java.util.Random rand = new java.util.Random();

    public static Translation3d[] visualizeFuelInHopper(
            Pose3d robotPose,
            int numFuelInRobot,
            double linearExtensionX,
            double verticalLift,
            double diagonalAngleRad,
            boolean isIntaking) {

        int count = Math.min(numFuelInRobot, SLOTS.length);
        Translation3d[] balls = new Translation3d[count];

        for (int i = 0; i < count; i++) {
            // MAGIC LOGIC:
            // When count is 70, ball 0 uses SLOT 0.
            // When count is 69 (one shot), the remaining ball 0 uses SLOT 0, 
            // but the ball that was at the very end of the intake (SLOT 69) is the one gone.
            
            // TO DO FIFO SHOOTING (Shooter leaves first, others slide):
            // We use the first 'count' slots, which are the ones closest to the shooter.
            Translation3d s = SLOTS[i];
            
            // Jitter for the 'intake' effect
            double jitterX = isIntaking ? (rand.nextDouble() - 0.5) * 0.005 : 0;
            double jitterY = isIntaking ? (rand.nextDouble() - 0.5) * 0.005 : 0;
            double jitterZ = isIntaking ? rand.nextDouble() * 0.008 : 0; 

            balls[i] = robotPose
                .plus(new Transform3d(
                    new Translation3d(s.getX() + jitterX, s.getY() + jitterY, s.getZ() + jitterZ),
                    Rotation3d.kZero))
                .getTranslation();
        }

        return balls;
    }

    public static Translation3d[] visualizeFuelInField() {
        return
            Arrays
                .stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"))
                .map(Pose3d::getTranslation)
                .toArray(Translation3d[]::new);
    }
}
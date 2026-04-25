package org.frogforce503.lib.rebuilt.sim;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class HopperFuelViz {
    private static final double D = Units.inchesToMeters(5.91);
    private static final double S = D * 0.72;                    

    private static final double START_X = -0.05; 
    private static final double FLOOR_Z  = 0.325;

    private static final int DEPTH  = 6;  
    private static final int WIDTH  = 5; 
    private static final int HEIGHT = 4;

    private static final int[] ROW_HEIGHTS = {4, 4, 4, 3, 2, 1};

    // --- NEW DATA STRUCTURE: SLOT CLASS ---
    private static class Slot {
        final double x, y, z;
        final int row, col, layer;
        boolean isFull = false;

        Slot(double x, double y, double z, int row, int col, int layer) {
            this.x = x; this.y = y; this.z = z;
            this.row = row; this.col = col; this.layer = layer;
        }
    }

    private static final List<Slot> allSlots = buildSlots();
    
    // Create two separate ordered lists for the state machine
    private static final List<Slot> intakeOrder = new ArrayList<>(allSlots);
    private static final List<Slot> shootOrder = new ArrayList<>(allSlots);
    
    // Track the previous count so we know whether we are adding or removing
    private static int lastCount = 0;
    private static final Random rand = new Random();

    static {
        // INTAKE ORDER: Fill Layer by Layer (Bottom to Top), then Row (Left to Right)
        intakeOrder.sort(Comparator.comparingInt((Slot s) -> s.layer)
                .thenComparingInt(s -> s.row)
                .thenComparingInt(s -> s.col));

        // SHOOT ORDER: Empty Row by Row (Left to Right), then Layer (Bottom to Top)
        // This ensures the entire Left Column empties before moving to the next.
        shootOrder.sort(Comparator.comparingInt((Slot s) -> s.row)
                .thenComparingInt(s -> s.layer)
                .thenComparingInt(s -> s.col));
    }

    private static List<Slot> buildSlots() {
        List<Slot> slots = new ArrayList<>();

        for (int layer = 0; layer < HEIGHT; layer++) {
            double z = FLOOR_Z + layer * S;
            
            for (int row = 0; row < DEPTH; row++) {
                if (layer >= ROW_HEIGHTS[row]) continue;

                double xOffset = (layer > 0) ? 0.1 : 0.0;
                double x = START_X + S / 2.0 + row * S + xOffset;
                
                for (int col = 0; col < WIDTH; col++) {
                    double y = (col - (WIDTH - 1) / 2.0) * S;
                    slots.add(new Slot(x, y, z, row, col, layer));
                }
            }
        }
        
        return slots;
    }

    private HopperFuelViz() {}

    public static Translation3d[] visualizeFuelInHopper(
            Pose3d robotPose,
            int numFuelInRobot,
            double linearExtensionX,
            double verticalLift,
            double diagonalAngleRad,
            boolean isIntaking) {

        int targetCount = Math.min(numFuelInRobot, allSlots.size());

        // --- STATE MACHINE ---
        // 1. If we gained balls (Intaking)
        while (lastCount < targetCount) {
            for (Slot s : intakeOrder) {
                if (!s.isFull) {
                    s.isFull = true;
                    lastCount++;
                    break;
                }
            }
        }

        // 2. If we lost balls (Shooting)
        while (lastCount > targetCount) {
            for (Slot s : shootOrder) {
                if (s.isFull) {
                    s.isFull = false;
                    lastCount--;
                    break;
                }
            }
        }

        // --- RENDERING ---
        List<Translation3d> balls = new ArrayList<>();
        
        for (Slot s : allSlots) {
            if (s.isFull) {
                double jitterX = isIntaking ? (rand.nextDouble() - 0.5) * 0.002 : 0;
                double jitterY = isIntaking ? (rand.nextDouble() - 0.5) * 0.002 : 0;
                double jitterZ = isIntaking ? rand.nextDouble() * 0.002 : 0; 

                balls.add(robotPose
                    .plus(new Transform3d(
                        new Translation3d(s.x + jitterX, s.y + jitterY, s.z + jitterZ),
                        Rotation3d.kZero))
                    .getTranslation());
            }
        }

        return balls.toArray(new Translation3d[0]);
    }
}
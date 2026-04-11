package org.frogforce503.lib.rebuilt;

import java.util.Arrays;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FuelViz {
    private static final int cols = 3;
    private static final int rows = 5;
    private static final int perLayer = cols * rows;
    private static final double fuelToFuelOffset = Units.inchesToMeters(4);
    private static final Transform3d robotToHopperOffset =
        new Transform3d(
            Units.inchesToMeters(3),
            Units.inchesToMeters(2),
            Units.inchesToMeters(9),
            Rotation3d.kZero);

    private FuelViz() {}

    public static Translation3d[] visualizeFuelInHopper(Pose3d robotPose, int numFuelInRobot) {
        Translation3d[] balls = new Translation3d[numFuelInRobot];

        for (int i = 0; i < numFuelInRobot; i++) {
            int layer = i / perLayer;
            int grid = i % perLayer;

            double x = (grid % cols - (cols - 1) / 2.0) * fuelToFuelOffset;
            double y = (grid / cols - (rows - 1) / 2.0) * fuelToFuelOffset;
            double z = layer * fuelToFuelOffset * 1.25; // slightly taller spacing for visibility

            balls[i] =
                robotPose
                    .plus(robotToHopperOffset)
                    .plus(new Transform3d(new Translation3d(x, y, z), Rotation3d.kZero))
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

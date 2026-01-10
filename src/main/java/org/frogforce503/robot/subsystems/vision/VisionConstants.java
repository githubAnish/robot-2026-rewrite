package org.frogforce503.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    /**
     * Cameras on robots are configured with a name.
     * Every camera on the robot must have a name from this enum.
     * This enum is used to identify specific cameras on a robot for any use case.
     */
    public enum CameraName {
        
    }

    public static final Matrix<N3, N1> DEFAULT_STANDARD_DEVIATIONS = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));
}

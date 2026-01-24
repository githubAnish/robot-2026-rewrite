package org.frogforce503.lib.rebuilt;

import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.constants.field.FieldConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility class to check if a hub is active. */
public final class AllianceHubUtil {
    private AllianceHubUtil() {}

    /** Checks if the specified alliance's hub is active.
     * See {@link https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html} for more.
     */
    public static boolean isHubActive(Alliance alliance) {
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length() <= 0) {
            System.out.println("No game data received yet" + ErrorUtil.attachJavaClassName(AllianceHubUtil.class));
            return false;
        }

        switch (gameData.charAt(0)) {
            case 'B':
                // Blue case code
                return alliance.equals(Alliance.Blue);
            case 'R':
                // Red case code
                return alliance.equals(Alliance.Red);
            default:
                // Corrupt data
                return false;
        }
    }

    /** Checks if our alliance's hub is active.
     * See {@link https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html} for more.
     */
    public static boolean isHubActive() {
        return isHubActive(FieldConstants.getAlliance());
    }
}
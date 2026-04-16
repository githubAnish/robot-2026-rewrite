package org.frogforce503.lib.rebuilt.sim;

import edu.wpi.first.math.MathUtil;

public final class FuelShotQuantityCalculator {
    private static final double fuelReleasedPerShot = 4; // How many balls are fired at once?

    private FuelShotQuantityCalculator() {}

    public static int computeFuelToShoot(int available) {
        if (available <= 0) {
            return 0;
        }

        double fillRatio = (double) available / 40.0; // Normalize (0 → 1)
        double curvedFill = Math.pow(fillRatio, 0.35); // Smooth curve
        double scaledMax = fuelReleasedPerShot * curvedFill; // Scale burst size

        // Bounds
        int minShot = Math.max(1, (int) Math.floor(scaledMax * 0.5));
        int maxShot = Math.max(1, (int) Math.ceil(scaledMax));

        // Weighted randomness
        double bias = curvedFill;
        double rand = Math.random();
        double weightedRand = (rand * (1 - bias)) + (Math.pow(rand, 0.5) * bias);

        int fuelToShoot = minShot + (int) (weightedRand * (maxShot - minShot + 1));

        // Simulate indexing inconsistency
        double misfeedChance = 0.15 * (1.0 - fillRatio);
        if (Math.random() < misfeedChance) {
            fuelToShoot -= 1;
        }

        double doubleFeedChance = 0.08 * fillRatio;
        if (Math.random() < doubleFeedChance) {
            fuelToShoot += 1;
        }

        double stutterChance = 0.1;
        if (Math.random() < stutterChance) {
            fuelToShoot += Math.random() < 0.5 ? -1 : 1;
        }

        return MathUtil.clamp(fuelToShoot, 1, available);
    }
}

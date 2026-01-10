package org.frogforce503.lib.vision.object_detection;

import java.util.Comparator;

/**
 * Enum representing different sorting modes for tracked objects.
 */
public enum ObjectSortingMode {
    Smallest(Comparator.comparingDouble(TrackedObject::area)), // Ascending order by area
    Largest(Smallest.comparator.reversed()), // Descending order by area
    Highest(Comparator.comparingDouble(TrackedObject::pitch)), // Ascending order by pitch (pitch is positive when tilted down)
    Lowest(Highest.comparator.reversed()), // Descending order by pitch
    Rightmost(Comparator.comparingDouble(TrackedObject::yaw)), // Ascending order by yaw (yaw is positive when turned left)
    Leftmost(Rightmost.comparator.reversed()), // Descending order by yaw
    Centermost(
            Comparator.comparingDouble(
                    target -> (Math.pow(target.pitch(), 2) + Math.pow(target.yaw(), 2)))), // Ascending order by distance from the center
    OuterMost(Centermost.comparator.reversed()); // Descending by distance from the center

    private final Comparator<TrackedObject> comparator;

    ObjectSortingMode(Comparator<TrackedObject> comparator) {
        this.comparator = comparator;
    }

    /**
     * Returns the comparator associated with this sorting mode.
     *
     * @return The comparator for sorting tracked objects.
     */
    public Comparator<TrackedObject> getComparator() {
        return comparator;
    }
}
package org.frogforce503.lib.vision.objectdetection;

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
                    target -> (Math.pow(target.pitch(), 2) + Math.pow(target.yaw(), 2))
            )
    ), // Ascending order by distance from the center

    OuterMost(Centermost.comparator.reversed()), // Descending by distance from the center

    Custom(Comparator.comparingDouble(target -> 0)); // Default comparator, can be set to anything by the user

    private Comparator<TrackedObject> comparator;

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

    /**
     * Sets the comparator for the Custom sorting mode. This allows users to define their own sorting logic.
     *
     * @param comparator The custom comparator to be used for sorting tracked objects when in Custom mode.
    */
    public void setComparator(Comparator<TrackedObject> comparator) {
        if (this == Custom) {
            this.comparator = comparator;
        }
    }
}
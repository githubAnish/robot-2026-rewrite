package org.frogforce503.lib.vision.apriltagdetection;

import java.util.Comparator;

import lombok.Getter;

/** Enum representing different sorting modes for tracked objects. */
public enum AprilTagSortingMode {
    Smallest(
        Comparator.comparingDouble(TrackedAprilTag::area)), // Ascending order by area

    Largest(
        Smallest.comparator.reversed()), // Descending order by area

    Highest(
        Comparator.comparingDouble(TrackedAprilTag::pitch)), // Ascending order by pitch (pitch is positive when tilted down)

    Lowest(
        Highest.comparator.reversed()), // Descending order by pitch

    Rightmost(
        Comparator.comparingDouble(TrackedAprilTag::yaw)), // Ascending order by yaw (yaw is positive when turned left)

    Leftmost(
        Rightmost.comparator.reversed()), // Descending order by yaw

    Centermost(
        Comparator.comparingDouble(
            target ->
                (Math.pow(target.pitch(), 2) + Math.pow(target.yaw(), 2)))), // Sorting by proximity to the center

    LeastAmbiguous(
        Comparator.comparingDouble(TrackedAprilTag::ambiguity)), // Ascending order by ambiguity
        
    MostAmbiguous(
        LeastAmbiguous.comparator.reversed()); // Descending order by ambiguity

    @Getter private final Comparator<TrackedAprilTag> comparator;

    private AprilTagSortingMode(Comparator<TrackedAprilTag> comparator) {
        this.comparator = comparator;
    }
}
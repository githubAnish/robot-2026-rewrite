package org.frogforce503.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a regular (equilateral and equiangular) polygon in a 2D plane.
 * This class provides methods to create, manipulate, and query properties of a regular polygon.
 * A regular polygon is defined by its center, radius, number of sides, and an optional rotation angle.
 */
public class Polygon2d {
    @Getter private final Translation2d center;
    @Getter private final double radius;
    @Getter private final int numSides;
    @Getter private final Rotation2d angleToRotate;

    @Getter private List<Translation2d> vertices = new ArrayList<>();

    // Method to generate vertices of a regular polygon
    public Polygon2d(Translation2d center, double radius, int numSides, Rotation2d angleToRotate) {
        this.center = center;
        this.radius = radius;
        this.numSides = numSides;
        this.angleToRotate = angleToRotate;

        double angleIncrement = 2 * Math.PI / numSides;

        for (int i = 0; i < numSides; i++) {
            double angle = i * angleIncrement + angleToRotate.getRadians();
            double x = center.getX() + radius * Math.cos(angle);
            double y = center.getY() + radius * Math.sin(angle);
            this.vertices.add(new Translation2d(x, y));
        }
    }

    public Polygon2d withNewCenter(Translation2d center) {
        return new Polygon2d(center, this.radius, this.numSides, this.angleToRotate);
    }

    public Polygon2d withNewRadius(double radius) {
        return new Polygon2d(this.center, radius, this.numSides, this.angleToRotate);
    }

    public Polygon2d withNewNumberOfSides(int numSides) {
        return new Polygon2d(this.center, this.radius, numSides, this.angleToRotate);
    }

    public Polygon2d withNewAngleToRotate(Rotation2d angleToRotate) {
        return new Polygon2d(this.center, this.radius, this.numSides, angleToRotate);
    }

    public boolean contains(Translation2d point) {
        int n = vertices.size();

        boolean result = false;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            if ((vertices.get(i).getY() > point.getY()) != (vertices.get(j).getY() > point.getY()) &&
                (point.getX() < (vertices.get(j).getX() - vertices.get(i).getX()) * (point.getY() - vertices.get(i).getY()) / (vertices.get(j).getY() - vertices.get(i).getY()) + vertices.get(i).getX())
            ) {
                result = !result;
            }
        }

        return result;
    }
}
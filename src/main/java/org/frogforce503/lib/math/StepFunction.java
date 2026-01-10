package org.frogforce503.lib.math;

import java.util.TreeMap;
import java.util.Map.Entry;

/** An alternative to a interpolating double tree map by using floor / ceiling entries as opposed to interpolation. */
public class StepFunction {
    private TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double x, double y) {
        map.put(x, y);
    }

    public double get(double lookupX) {
        return getFloorEntry(lookupX);
    }
    
    public double getFloorEntry(double lookupX) {
        Entry<Double, Double> floor = map.floorEntry(lookupX);

        if (floor != null) {
            return floor.getValue();
        }

        return map.get(map.firstKey());
    }

    public double getCeilingEntry(double lookupX) {
        Entry<Double, Double> ceil = map.ceilingEntry(lookupX);

        if (ceil != null) {
            return ceil.getValue();
        }

        return map.get(map.firstKey());
    }
}

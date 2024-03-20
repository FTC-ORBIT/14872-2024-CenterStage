package org.firstinspires.ftc.teamcode.OrbitUtils;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public final class MathFuncs {
    public static Double max(double... doubleList) {
        double biggest =0;
        for(double i : doubleList) {
            if (i > biggest) {
                biggest = i;
            }
        }
        return biggest;
    }
    public static Double max(Object... objectList) {
        double biggest =0;
        double[] doubleList = new double[objectList.length];
        for(int i=0 ; i < objectList.length ; i++ ) {
            doubleList[i] = (double) objectList[i];
        }
        for(double i : doubleList) {
            if (i > biggest) {
                biggest = i;
            }
        }
        return biggest;
    }

    public static float deadBand(final float val, final float xAtZero, final float xAtOne) {
        final float returnVal = (Math.abs(val) - xAtZero) / (xAtOne - xAtZero);
        return range(0, 1, returnVal) * Math.signum(val);
    }

    public static float pointAndSlopeLinear(final Vector point, final float slope, final float value) {
        return slope * (value - point.x) + point.y;
    }

    public static float twoPointsLinear(final Vector pointA, final Vector pointB, final float value) {
        return (pointA.x == pointB.x) ? Constants.INF
                : pointAndSlopeLinear(pointA, (pointA.y - pointB.y) / (pointA.x - pointB.x), value);
    }

    public static float deadBand(final Vector point, final float slope, final float minVal, final float maxVal,
            final float value) {
        return range(minVal, maxVal, pointAndSlopeLinear(point, slope, value));
    }

    public static float deadBand(final Vector a, final Vector b, final float value) {
        final float maxValue = Math.max(a.y, b.y);
        final float minValue = Math.min(a.y, b.y);
        return range(minValue, maxValue, twoPointsLinear(a, b, value));
    }

    public static float range(final float lowerBound, final float upperBound, final float value) {
        return Math.min(Math.max(lowerBound, value), upperBound);
    }

    public static float limit(final float bound, final float value) {
        return range(-Math.abs(bound), Math.abs(bound), value);
    }

    public static boolean inRange(final float value, final float lowerBound, final float upperBound) {
        return value <= upperBound && value >= lowerBound;
    }

    public static boolean inTolerance(final float value, final float wantedValue, final float tolerance) {
        return inRange(value, wantedValue - tolerance, wantedValue + tolerance);
    }

    public static float relativeDifference(final float value, final float reference) {
        return Math.abs((value - reference) / reference);
    }

    private static int[] factorials = { 1, 1, 2, 6 };

    public static float hypotenuse(final float a, final float b) {
        return (float) Math.sqrt(a * a + b * b);
    }
}
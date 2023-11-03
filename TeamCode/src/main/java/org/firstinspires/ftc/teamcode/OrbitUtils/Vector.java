package org.firstinspires.ftc.teamcode.OrbitUtils;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public final class Vector {

    public float x;
    public float y;

    public Vector(final float x, final float y) {
        this.x = x;
        this.y = y;
    }

    public static Vector zero() {
        return new Vector(0, 0);
    } // returns Vector zero

    public static Vector INF() {
        return new Vector(Constants.INF, Constants.INF);
    } // returns INF Vector

    public float norm() {
        return (float) Math.sqrt(x * x + y * y);
    }

    public float getAngle() {
        if (x == 0 && y == 0) {
            return 0;
        } else {
            return (float) Math.atan2(y, x);
        }
    }

    public Vector scale(final float scalingFactor) {
        return new Vector(x * scalingFactor, y * scalingFactor);
    }

    public Vector unit() {
        if (x == 0 && y == 0) {
            return this;
        } else {
            return scale(1 / norm());
        }
    }

    // returns maagal hyehida
    public static Vector unit(final float angle) {
        return new Vector((float) Math.cos(angle), (float) Math.sin(angle));
    }

    public static Vector fromAngleAndRadius(final float theta, final float radius) {
        final float vectorX = (float) Math.cos(theta) * radius;
        final float vectorY = (float) Math.sin(theta) * radius;
        return new Vector(vectorX, vectorY);
    }

    public float dotProduct(final Vector other) {
        return x * other.x + y * other.y;
    }

    public Vector rotate(final float theta) {
        final float sinTheta = (float) Math.sin(theta);
        final float cosTheta = (float) Math.cos(theta);

        final float newX = x * cosTheta - y * sinTheta;
        final float newY = x * sinTheta + y * cosTheta;

        return new Vector(newX, newY);
    }

    public Vector rotate90(final boolean rotateCounterClockwise) {
        if (!rotateCounterClockwise) {
            return new Vector(-y, x);
        } else {
            return new Vector(y, -x);
        }
    }

    public Vector abs() {
        return new Vector(Math.abs(x), Math.abs(y));
    }

    public Vector add(final Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector subtract(final Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public float project(final float angle) {
        final Vector unitVector = unit(angle);
        return dotProduct(unitVector);
    }

    public float project(final Vector other) {
        final float otherNorm = other.norm();
        return otherNorm != 0 ? dotProduct(other) / otherNorm : this.norm();
    }

    public static Vector longest(final Vector a, final Vector b) {
        return a.norm() > b.norm() ? a : b;
    }

    public static Vector shortest(final Vector a, final Vector b) {
        return a.norm() < b.norm() ? a : b;
    }

    public static Vector max(final Vector a, final Vector b) {
        return new Vector(Math.max(a.x, b.x), Math.max(a.y, b.y));
    }

    public static Vector min(final Vector a, final Vector b) {
        return new Vector(Math.min(a.x, b.x), Math.min(a.y, b.y));
    }

    public static Vector sameXY(final float value) {
        return new Vector(value, value);
    }

    @Override
    public String toString() {
        return "x: " + x + " y: " + y;
    }

    public boolean equals(final Vector other) {
        return x == other.x && y == other.y;
    }

    public static float angleDifference(final Vector a, final Vector b) {
        final float normA = a.norm();
        final float normB = b.norm();

        if (normA == 0 || normB == 0)
            return 0;

        return (float) Math.acos(a.dotProduct(b) / (normA * normB));
    }
}

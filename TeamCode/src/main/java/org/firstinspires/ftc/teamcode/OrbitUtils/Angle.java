package org.firstinspires.ftc.teamcode.OrbitUtils;

public final class Angle {

    public static final float halfPI = (float) Math.PI / 2;
    public static final float pi = (float) Math.PI;
    public static final float twoPI = (float) Math.PI * 2;

    public static float wrapAngle0_2PI(final float theta) {
        return (theta % twoPI + twoPI) % twoPI;
    }
    // convert 0-360 iRadians

    public static float wrapAnglePlusMinusPI(final float theta) {
        final float wrapped = theta % twoPI; // Getting the angle smallest form (not exceeding a
                                             // full turn).
// convert -180-180 in Radians
        // Adding or subtracting two pi to fit the range of +/- pi.
        if (wrapped > pi) {
            return wrapped - twoPI;
        } else if (wrapped < -pi) {
            return wrapped + twoPI;
        } else {
            return wrapped;
        }
    }

    // Functions to convert between degrees and radians:
    public static float degToRad(final float theta) {
        return (float) Math.toRadians(theta);
    } //float

    public static float radToDeg(final float theta) {
        return (float) Math.toDegrees(theta);
    } //float

    public static float projectBetweenPlanes(final float theta, final float alpha) {
        if (alpha < 0) {
            return (float) Math.atan(Math.tan(theta) / Math.cos(alpha));
        } else {
            return (float) Math.atan(Math.tan(theta) * Math.cos(alpha));
        }
    }
    // alpha is the angle between the planes.
    // For more information check:
    // https://math.stackexchange.com/questions/2207665/projecting-an-angle-from-one-plane-to-another-plane
}

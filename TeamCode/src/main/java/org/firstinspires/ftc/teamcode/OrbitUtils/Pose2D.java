package org.firstinspires.ftc.teamcode.OrbitUtils;

public class Pose2D {
    public Vector translation;
    public float rotation;

    public static Pose2D zero() {
        return new Pose2D(Vector.zero(), 0);
    }

    public Pose2D(final Vector translation, final float rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Pose2D subtract(final Pose2D other) {
        return new Pose2D(this.translation.subtract(other.translation),
                Angle.wrapAnglePlusMinusPI(this.rotation - other.rotation));
    }

    @Override
    public String toString() {
        return translation.toString() + " dir: " + Angle.radToDeg(rotation);
    }
}

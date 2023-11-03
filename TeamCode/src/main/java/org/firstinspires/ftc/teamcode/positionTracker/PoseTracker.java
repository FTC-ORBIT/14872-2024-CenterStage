package org.firstinspires.ftc.teamcode.positionTracker;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class PoseTracker {
    private static Pose2d robotPose;
    private static Vector robotVelocity;
    private static Vector robotAcceleration;
    private static float omega;
    private static float lastAngle = (float) robotPose.getHeading();

    public static void calcPose() { // * we are calling this function every cycle in the opmode.
        robotPose = Drivetrain.getPose_FieldCS();
        calcPoseDerivatives();
    }

    public static void calcPoseDerivatives() {
        // TODO: here the velocity and acceleration will go.
        robotVelocity = getVelocity();
        robotAcceleration = getAcceleration();

    }

    public static Pose2d getPose() {
        return robotPose;
    }

    public static Vector getPosition() {
        return new Vector((float) robotPose.getX(), (float) robotPose.getY());
    }

    public static float getHeading() {
        return (float) robotPose.getHeading();
    }

    public static float getOmega() {
        final float omega = (getHeading() - lastAngle) / GlobalData.deltaTime;
        lastAngle = getHeading();
        return omega;
    }

    public static Vector getVelocity() {
        return Drivetrain.getVelocity_FieldCS();
    }

    public static Vector getAcceleration() {
        return Drivetrain.getAcceleration();
    }

    public static float getSpeed() {
        return getVelocity().norm();
    }

    public static void setPose(Pose2d pose2d) {
        robotPose = pose2d;
    }

    public static void resetPosition() {
        setPose(new Pose2d(0, 0, 0));
    }

}

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    public static double endTangent = 60;
    public static double x = 75;
    public static double y = 0;
    public static double endAngle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0, 0,0))
                .splineToLinearHeading(new Pose2d(x, y, Math.toRadians(endAngle)), Math.toRadians(endTangent))
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(traj);

    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import android.widget.Switch;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThreshold;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueCloseToTheBoard")
@Config
public class BlueCloseToTheBoard extends LinearOpMode {
    public static double driveToConeX = 29.5;
    public static double goToParkingY = -38;

    public static double leftAngle = 0;
    public static double rightConeX = 22.5;

    public static double rightConeY = -8;

    public static double leftDriveX = 13;
    public static double leftConeX = 22.5;

    public static double leftConeY = 11;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);



        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(driveToConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeX, rightConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftDriveX  , startPose.getY() , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftConeX,leftConeY,leftAngle))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(leftCone);
        }
    }
}

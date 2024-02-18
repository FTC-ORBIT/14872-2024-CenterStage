package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueFarFromTheBoard")
@Config
public class BlueFarFormTheBoard extends  LinearOpMode{
    public static double centerConeX = 29.5;

    public static double parkingY = -88;

    public static double rightConeX = 22.5;

    public static double rightConeY = -8;


    public static double leftConeX = 22.5;

    public static double leftConeY = 7;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);


        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY, startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeX, rightConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY, startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftConeX, leftConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY() ,startPose.getHeading() ))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY, startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(leftCone);
        }
    }
}

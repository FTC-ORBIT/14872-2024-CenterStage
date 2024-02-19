package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThresholdClose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "BlueFarFromTheBoard")
@Config
public class BlueFarFormTheBoard extends  LinearOpMode{
    public static double centerConeX = 29.5;

    public static double parkingY = -88;
    public static double rightDriveX = 24.9;
    public static double rightConeX = 29.06;

    public static double rightConeY = -5.2;

    public static double rightConeAngle =4.7;
    public static double leftConeX = 22.5;

    public static double leftConeY = 7;
    private VisionPortal portal;
    private BluePropThresholdClose bluePropThresholdClose = new BluePropThresholdClose();

    @Override
    public void runOpMode() throws InterruptedException{

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdClose)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);


        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY, startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightDriveX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(rightConeX , rightConeY , rightConeAngle))
                .lineToLinearHeading(new Pose2d(rightConeX , rightConeY+5, rightConeAngle))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY() , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY, startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftConeX, leftConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY() ,startPose.getHeading() ))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY, startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            switch (bluePropThresholdClose.blueEnumGetPropPos()) {
            case LEFT:
                drive.followTrajectorySequence(leftCone);
                telemetry.addLine("left");
                break;
            case CENTER:
                drive.followTrajectorySequence(centerCone);
                telemetry.addLine("center");
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightCone);
                telemetry.addLine("right");
                break;
            case NONE:
                telemetry.addLine("Doesn't see prop");
                break;
        }
            telemetry.update();
        }        }
    }


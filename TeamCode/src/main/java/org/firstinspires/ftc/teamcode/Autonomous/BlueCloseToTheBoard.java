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

@Autonomous(name = "BlueCloseToTheBoard")
@Config
public class BlueCloseToTheBoard extends LinearOpMode {
    public static double driveToConeX = 29.5;
    public static double goToParkingY = 38;
    public static double delay = 3;
    public static double leftAngle = -1.433;
    public static double rightConeX = 22.5;

    public static double rightConeY = 9;

    public static double leftDriveX = 22.02;
    public static double leftConeX = 26.5;
    public static double leftConeY = -4;


    private VisionPortal portal;
    private BluePropThresholdClose bluePropThresholdClose = new BluePropThresholdClose();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        bluePropThresholdClose.initProp();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdClose)
                .build();

        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(driveToConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getHeading()))
                                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(new Pose2d(leftDriveX  , startPose.getY() , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftConeX,leftConeY,leftAngle))
                                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();


        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeX, rightConeY, startPose.getHeading()))
                .lineToLinearHeading(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();
        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX() + 10, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 10, startPose.getY() + 10, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX(), startPose.getY() + 10, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading()))
                        .build();
        waitForStart();

        if (!isStopRequested()) {
            switch (bluePropThresholdClose.EnumGetPropPos()) {
                case LEFT:
                    drive.followTrajectorySequence(leftCone);
                    telemetry.addLine("left");
                    telemetry.update();
                    break;
                case CENTER:
                    drive.followTrajectorySequence(centerCone);
                    telemetry.addLine("center");
                    telemetry.update();
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(rightCone);
                    telemetry.addLine("right");
                    telemetry.update();
                    break;
                case NONE:
                    telemetry.addLine("Doesn't see prop");
                    telemetry.update();
                    break;
            }
        }
    }
}

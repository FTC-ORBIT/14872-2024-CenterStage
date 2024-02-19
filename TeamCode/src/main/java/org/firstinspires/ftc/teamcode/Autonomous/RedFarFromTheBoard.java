package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThreshold;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "RedFarFromTheBoard")
@Config
public class RedFarFromTheBoard extends  LinearOpMode{
    public static double centerConeXRed = 29.5;

    public static double parkingYRed = 88;

    public static double rightConeXRed = 22.5;

    public static double rightConeYRed = 8;


    public static double leftConeXRED = 22.5;

    public static double leftConeYRed = -7;
    private VisionPortal portal;
    private RedPropThreshold redPropThreshold = new RedPropThreshold();

    @Override
    public void runOpMode() throws InterruptedException{

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(redPropThreshold)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);


        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeXRed, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingYRed, startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeXRed, rightConeYRed, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingYRed, startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftConeXRED, leftConeYRed, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY() ,startPose.getHeading() ))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingYRed, startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            switch (redPropThreshold.redEnumGetPropPos()) {
                case LEFT:
                    drive.followTrajectorySequence(leftCone);
                    break;
                case CENTER:
                    drive.followTrajectorySequence(centerCone);
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(rightCone);
                    break;
                case NONE:
                    telemetry.addLine("Doesn't see prop");
                    break;
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "RedFarFromTheBoard")
@Config
public class RedFarFormTheBoard extends  LinearOpMode{
    public static double centerConeX = 29.5;
    public static double delay = 3000;
    public static double parkingY = -88;
    public static double boardY = -78;
    public static double rightDriveX = 27;
    public static double rightConeX = 29.06;

    public static double rightConeY = -5.2;

    public static double rightConeAngle =5.05;
    public static double leftConeX = 22.5;

    public static double leftConeY = 7;
    private VisionPortal portal;
    private RedPropThresholdFar redPropThresholdFar = new RedPropThresholdFar();

    @Override
    public void runOpMode() throws InterruptedException{
        OrbitGyro.init(hardwareMap);
        Elevator.init(hardwareMap);
        Outtake.init(hardwareMap);
        Intake.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Plane.init(hardwareMap);

        redPropThresholdFar.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(redPropThresholdFar)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);


        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY , startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightDriveX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(rightConeX , rightConeY , rightConeAngle))
                .lineToLinearHeading(new Pose2d(rightConeX , startPose.getY() , rightConeAngle))
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
            sleep((long) delay);
            switch (redPropThresholdFar.EnumGetPropPos()) {
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


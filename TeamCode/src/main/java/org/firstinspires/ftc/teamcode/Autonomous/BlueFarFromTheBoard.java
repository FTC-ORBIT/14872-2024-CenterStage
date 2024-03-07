package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "BlueFarFromTheBoard")
@Config
public class BlueFarFromTheBoard extends  LinearOpMode{
    public static double centerConeX = 29.5;
    public static double delay = 3;
    public static double parkingY = -88;
    public static double rightDriveX = 27;
    public static double rightConeX = 29.06;

    public static double rightConeY = 5;
    public static double boardY = 84;
    public static double rightConeAngle =5.05;
    public static double leftConeX = 22.5;

    public static double leftConeY = -7.3;

    public static double centerAfterConeX = 23;
    public static double centerAfterConeY = 18;
    public static double centerGateX= 49.07;
    public static double centerGateY = -16.41;
    public static double afterGateX = 53;
    public static double afterGateY = 70.345;
    public static double boardPos34  = 26.3;
    public static double markerY = 70;
    private VisionPortal portal;
    private BluePropThresholdFar bluePropThresholdFar = new BluePropThresholdFar();

    @Override
    public void runOpMode() throws InterruptedException{

        bluePropThresholdFar.initProp();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdFar)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerConeX - 4, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerAfterConeX, centerAfterConeY ,startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerGateX , centerGateY ,startPose.getHeading()))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(centerGateX + 3, centerGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(boardPos34, markerY,Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(boardPos34 , boardY , Math.toRadians(90)))
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .lineToLinearHeading(new Pose2d(boardPos34, markerY,Math.toRadians(90)))

                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.CLOSED);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateTeleop(FourbarState.REVERSE);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.INTAKE, telemetry);
                })
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, parkingY , startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftConeX, leftConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY() ,startPose.getHeading() ))
                                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, -parkingY, startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
         .lineToLinearHeading(new Pose2d(rightDriveX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(rightConeX , rightConeY , -rightConeAngle))
                .lineToLinearHeading(new Pose2d(rightConeX , startPose.getY() , -rightConeAngle))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, startPose.getY() , startPose.getHeading()))
                                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, -parkingY, startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            sleep((long) delay);
            switch (bluePropThresholdFar.EnumGetPropPos()) {
                case LEFT:
                    telemetry.addLine("left");
                    telemetry.update();
                    drive.followTrajectorySequence(leftCone);
                    break;
                case CENTER:
                    telemetry.addLine("center");
                    telemetry.update();
                    drive.followTrajectorySequence(centerCone);
                    break;
                case RIGHT:
                    telemetry.addLine("right");
                    telemetry.update();
                    drive.followTrajectorySequence(rightCone);
                    break;
                case NONE:
                    telemetry.addLine("Doesn't see prop");
                    telemetry.update();
                    break;
            }
            telemetry.update();
        }        }
}


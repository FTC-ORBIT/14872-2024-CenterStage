package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Far Far Wall")
@Config
public class RedFarFromTheBoard extends  LinearOpMode{
    public static double maxVeloDrop = 6.5;
    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);
    public static double centerConeX = 28;
    public static double delay = 5000;
    public static double parkingY = -88;
    public static double parkingX = 46.19;
    public static double boardY = -86;
    public static double rightDriveX = 27;
    public static double rightConeX = 29.06;

    public static double rightConeY = -5.2;

    public static double rightConeAngle =5.05;
    public static double leftConeX = 22.5;

    public static double leftConeY = 9;
    public static double centerAfterConeX = 23;
    public static double centerAfterConeY = 18;
    public static double centerGateX= 49.07;
    public static double centerGateY = 16.41;
    public static double afterGateX = 53;
    public static double afterGateY = -70.345;
    public static double boardPos12 = 37;
    public static double boardPos34 = 30;
    public static double boardPos56 = 23.3;
    // TODO X6 = 22.5
    // TODO X5 = 23.3
    // TODO X4 = 28.28
    // TODO X3 = 30.0
    // TODO X2 = 34.5
    // TODO X1 = 37
    public static double leftAfterPropX = 16;
    public static double leftAfterPropY = -3.5;
    public static double leftBeforeGateX =48;
    public static double rightAfterPropY = 3;
    public static double rightBeforeGateX = 52.8;
    public static double markerY = -77;

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
                .lineToLinearHeading(new Pose2d(centerConeX - 4, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerAfterConeX, centerAfterConeY ,startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerGateX , centerGateY ,startPose.getHeading()))
                .turn(Math.toRadians(-90))
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(centerGateX + 3, centerGateY , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(boardPos34, markerY,Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos34 , boardY , Math.toRadians(-90)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos34, markerY,Math.toRadians(-90)))

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
                .turn(startPose.getHeading())
                .lineToLinearHeading(new Pose2d(parkingX, parkingY , startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(rightDriveX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(rightConeX , rightConeY , rightConeAngle))
                .lineToLinearHeading(new Pose2d(rightConeX , startPose.getY() , rightConeAngle))
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(rightConeX, rightAfterPropY, rightConeAngle))
                .lineToLinearHeading(new Pose2d(rightBeforeGateX, rightAfterPropY , rightConeAngle))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(boardPos56, markerY,Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos56 , boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos56, markerY , Math.toRadians(-90)))
                .waitSeconds(1)
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
                .turn(startPose.getHeading())
                .lineToLinearHeading(new Pose2d(parkingX, parkingY , startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftConeX, leftConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, leftConeY , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftAfterPropX, leftAfterPropY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftBeforeGateX, leftAfterPropY , startPose.getHeading()))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(leftBeforeGateX + 5, leftAfterPropY , Math.toRadians(-90)))
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(boardPos12, markerY,Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos12 , boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(2)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos12,markerY,Math.toRadians(-90)))
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
                .turn(startPose.getHeading())
                .lineToLinearHeading(new Pose2d(parkingX, parkingY , startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            switch (redPropThresholdFar.EnumGetPropPos()) {
                case LEFT:
                    drive.followTrajectorySequence(leftCone);
                    telemetry.addLine("left");
                    telemetry.update();
                    break;
                case CENTER:
                default:
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


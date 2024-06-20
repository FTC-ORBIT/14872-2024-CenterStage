package org.firstinspires.ftc.teamcode.Autonomous;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.AprilTagDetect;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.BluePropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.enums.YellowPixelPosEnum;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Blue Far Far Wall",group = "Blue")
@Config
public class BlueFarFromTheBoard extends  LinearOpMode{
    public static double timerCount;
    public static double maxVeloDrop = 8;
    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);
    public static double centerConeX = 29.7;
    public static double delay = 5000;
    public static double parkingY = 83;
    public static double parkingX = 46.19;
    public static double boardY = 85;
    public static double rightConeX = 33;

    public static double rightConeY = 4.3;


    public static double prepareToPropY = 1;

    public static double rightConeAngle =5.05;
    public static double leftConeX = 22.5;

    public static double leftConeY = -9;
    public static double centerAfterConeX = 23;
    public static double centerAfterConeY = -18;
    public static double centerGateX= 49.07;

    public static double centerGateSecondx = 55.07;
    public static double centerGateY = -16.41;
    public static double afterGateX = 55;
    public static double afterGateY = 70.345;
    public static double boardPos1 = 37 ;
    public static double boardPos2 = 35;
    public static double boardPos3HitLeft = 32.2;
    public static double boardPos3MissRight = 31;
    public static double boardPos4MissLeft = 31;
    public static double boardPos4HitRight = 29;
    public static double boardPos5 = 23.6;
    public static double boardPos6 = 21.9;
    // TODO X6 = 22.1
    // TODO X5 = 26
    // TODO X4 = 29.2
    // TODO X3 = 32
    // TODO X2 = 34.3
    // TODO X1 = 37
    public static double leftAfterPropX = 16;
    public static double leftAfterPropY = 3;
    public static double leftBeforeGateX =48;
    public static double rightAfterPropY = -3;
    public static double rightBeforeGateX = 52.8;
    public static double markerY = 77;
    public static double markerX = 35.4;
    public static double leftMarkerX = 28;

    public static double rightEndTangent = -30;
    public static ElevatorStates state = ElevatorStates.AUTO;
    private VisionPortal portal;
    private BluePropThresholdFar bluePropThresholdFar = new BluePropThresholdFar();
    private AprilTagDetect aprilTag;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException{

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(1, 0, 0);

        drive.setPoseEstimate(startPose);

        Elevator.init(hardwareMap);
        Outtake.init(hardwareMap);
        Intake.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Plane.init(hardwareMap);

        aprilTag = new AprilTagDetect();
        aprilTag.atPrcsr = new AprilTagProcessor.Builder()
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(458.066, 457.626, 337.176, 251.805)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        bluePropThresholdFar.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag.atPrcsr)
                .addProcessor(bluePropThresholdFar)
                .build();

        FtcDashboard.getInstance().startCameraStream(portal, 30);

        AprilTagDetect.setManualExposure(4,30, portal, this);

        portal.setProcessorEnabled(aprilTag.atPrcsr,false);


        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerConeX - 4, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerAfterConeX, centerAfterConeY ,startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerGateX , centerGateY ,startPose.getHeading()))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(centerGateSecondx, centerGateY , Math.toRadians(90)))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(markerX, markerY,Math.toRadians(90)))
//                .addTemporalMarker(() ->{
//                    redPropThresholdFar.initYellowPixel();
//                })
                .waitSeconds(1)
                .build();

        TrajectorySequence centerConeHitL = drive.trajectorySequenceBuilder(centerCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO,  telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1.5)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos3HitLeft, boardY , Math.toRadians(90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(0.3)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos3HitLeft, markerY,Math.toRadians(90)))

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

        TrajectorySequence centerConeHitR = drive.trajectorySequenceBuilder(centerCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO,  telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1.5)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos4HitRight, boardY , Math.toRadians(90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(0.3)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos4HitRight, markerY,Math.toRadians(90)))

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

        TrajectorySequence centerConeMissL = drive.trajectorySequenceBuilder(centerCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO,  telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1.5)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos4MissLeft, boardY , Math.toRadians(90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(1)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos4MissLeft, markerY,Math.toRadians(90)))

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

        TrajectorySequence centerConeMissR = drive.trajectorySequenceBuilder(centerCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO,  telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1.5)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos3MissRight, boardY , Math.toRadians(90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(1)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos3MissRight, markerY,Math.toRadians(90)))

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

        TrajectorySequence centerConeNopPixel = drive.trajectorySequenceBuilder(centerCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN,  telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1.5)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos4HitRight, boardY , Math.toRadians(90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(0.3)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos3HitLeft, markerY,Math.toRadians(90)))

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
                .splineToLinearHeading(new Pose2d(rightConeX, prepareToPropY, Math.toRadians(90)), Math.toRadians(30))
                .splineToLinearHeading(new Pose2d(rightConeX, rightConeY, Math.toRadians(90)), Math.toRadians(30))
                .lineToLinearHeading(new Pose2d(rightConeX, rightAfterPropY, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(rightBeforeGateX, rightAfterPropY , Math.toRadians(90)))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(leftMarkerX, markerY,Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(boardPos5, markerY, Math.toRadians(-90)), Math.toRadians(rightEndTangent))
//                .addTemporalMarker(() -> {
//                    redPropThresholdFar.initYellowPixel();
//                })
                .waitSeconds(1)
                .build();

        TrajectorySequence rightConeHitL = drive.trajectorySequenceBuilder(rightCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);

                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos5, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos5, markerY , Math.toRadians(90)))
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

        TrajectorySequence rightConeHitR  = drive.trajectorySequenceBuilder(rightCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos6, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos6, markerY , Math.toRadians(90)))
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

        TrajectorySequence rightConeMissL = drive.trajectorySequenceBuilder(rightCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos5, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos5, markerY , Math.toRadians(90)))
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

        TrajectorySequence rightConeNopPixel = drive.trajectorySequenceBuilder(rightCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos6, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(boardPos6, markerY , Math.toRadians(90)))
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
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(leftBeforeGateX + 5, leftAfterPropY , Math.toRadians(90)))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(markerX, markerY,Math.toRadians(90)))
//                .addTemporalMarker(() ->{
//                    redPropThresholdFar.initYellowPixel();
//                })
                .waitSeconds(1)
                .build();

        TrajectorySequence leftConeHitL = drive.trajectorySequenceBuilder(leftCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos1, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos1,markerY,Math.toRadians(90)))
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

        TrajectorySequence leftConeHitR = drive.trajectorySequenceBuilder(leftCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos2, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(2)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos2,markerY,Math.toRadians(90)))
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

        TrajectorySequence leftConeMissR = drive.trajectorySequenceBuilder(leftCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos2, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos2,markerY,Math.toRadians(90)))
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

        TrajectorySequence leftConeNopPixel = drive.trajectorySequenceBuilder(leftCone.end())
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos1, boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos1,markerY,Math.toRadians(90)))
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
        while (!bluePropThresholdFar.completedPropPos){
            telemetry.addLine("waiting for camera to start");
            telemetry.update();
            sleep(1);
        }
        if (!isStopRequested()) {
//            redPropThresholdFar.initYellowPixel();
            bluePropThresholdFar.EnumGetPropPos();
            portal.setProcessorEnabled(aprilTag.atPrcsr, true);
            portal.setProcessorEnabled(bluePropThresholdFar, false);

            switch (bluePropThresholdFar.sampledPropPos) {
                case LEFT: // this is the same code as Red - right cone
                    drive.followTrajectorySequence(rightCone);
                    getYellowPixelfromAprilTag();
//                    while (!gamepad1.b){
//                        sleep(5);
//                        telemetry.addData("pixel in",bluePropThresholdFar.sampledYellowPixelPos);
//                        telemetry.addData("prop in", bluePropThresholdFar.sampledPropPos);
//                        telemetry.addData("yellow Pixels", bluePropThresholdFar.biggest.averagedBox);
//                        telemetry.update();
//                    }
                    chooseDropTraj(rightConeHitL, rightConeHitR, rightConeNopPixel, rightConeMissL, rightConeNopPixel);
                    telemetry.addData("pixel in",bluePropThresholdFar.sampledYellowPixelPos);
                    telemetry.addData("prop in", bluePropThresholdFar.sampledPropPos);
                    telemetry.update();
                    sleep(10000);
                    break;
                case CENTER:
                default:
                    drive.followTrajectorySequence(centerCone);
                    getYellowPixelfromAprilTag();
                    chooseDropTraj(centerConeHitL, centerConeHitR, centerConeMissL, centerConeMissR, centerConeNopPixel);
//                    if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITLEFT) {
//                     drive.followTrajectorySequence(centerConeHitL);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITRIGHT){
//                     drive.followTrajectorySequence(centerConeHitR);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSLEFT){
//                     drive.followTrajectorySequence(centerConeMissL);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSRIGHT){
//                     drive.followTrajectorySequence(centerConeMissR);
//                    }else {
//                     drive.followTrajectorySequence(centerConeNopPixel);
//                    }
                    telemetry.addData("pixel in",bluePropThresholdFar.sampledYellowPixelPos);
                    telemetry.addData("prop in", bluePropThresholdFar.sampledPropPos);
                    telemetry.update();
                    sleep(10000);
                    break;
                case RIGHT: // this is the same code as Red - left cone
                    drive.followTrajectorySequence(leftCone);
                    getYellowPixelfromAprilTag();
                    chooseDropTraj(leftConeHitL,leftConeHitR,leftConeMissR,leftConeNopPixel,leftConeNopPixel);
                    telemetry.addData("pixel in",bluePropThresholdFar.sampledYellowPixelPos);
                    telemetry.addData("prop in", bluePropThresholdFar.sampledPropPos);
                    telemetry.update();
                    sleep(10000);
                    break;
                case NONE:
                    drive.followTrajectorySequence(centerCone);
                    getYellowPixelfromAprilTag();
                    chooseDropTraj(centerConeHitL, centerConeHitR, centerConeMissL, centerConeMissR, centerConeNopPixel);
                    telemetry.addData("pixel in",bluePropThresholdFar.sampledYellowPixelPos);
                    telemetry.addData("prop in", bluePropThresholdFar.sampledPropPos);
                    telemetry.update();
                    sleep(10000);
                    telemetry.addLine("not");
                    break;
            }
            while (!isStopRequested()) {
                print_tele(10, true);
            }
        }
    }

    public void print_tele (long millisec, boolean test){
        telemetry.addData("prop pos:", bluePropThresholdFar.EnumGetPropPos());
        telemetry.addData("yellow pixel:", bluePropThresholdFar.sampledYellowPixelPos);
        if(bluePropThresholdFar.biggest != null) {
            telemetry.addData("biggest:", bluePropThresholdFar.biggest.averagedBox);
        }
        telemetry.addData("wantedID", aprilTag.wantedID);
        telemetry.addData("Detect attempts", aprilTag.count);
        telemetry.update();

        if (bluePropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.NOPIXEL){
             while(!isStopRequested()){
             sleep(1);
            }
        }
        sleep(millisec);
        if (test) {
            bluePropThresholdFar.test(gamepad1, telemetry);
        }
    }

    public void getYellowPixelfromAprilTag(){
        portal.resumeStreaming();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING){
            sleep(1);
        }
        aprilTag.getAprilTagCords(bluePropThresholdFar.sampledPropPos,
                                  bluePropThresholdFar.AllianceColor);
        bluePropThresholdFar.initYellowPixelAT(aprilTag.aprilTagCords);
        AprilTagDetect.setDfltExposure(portal,this);
//        AprilTagDetect.setManualExposure(15, 30, portal, this);
        portal.setProcessorEnabled(bluePropThresholdFar, true);
//        sleep(100);
        bluePropThresholdFar.getYellowPixelPos();
    }

    public void chooseDropTraj(TrajectorySequence ConeHitL,
                               TrajectorySequence ConeHitR,
                               TrajectorySequence ConeMissL,
                               TrajectorySequence ConeMissR,
                               TrajectorySequence ConeNopPixel){
        if (bluePropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITLEFT) {
            drive.followTrajectorySequence(ConeHitL);
        } else if (bluePropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITRIGHT){
            drive.followTrajectorySequence(ConeHitR);
        } else if (bluePropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSLEFT){
            drive.followTrajectorySequence(ConeMissL);
        } else if (bluePropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSRIGHT){
            drive.followTrajectorySequence(ConeMissR);
        } else {
            drive.followTrajectorySequence(ConeNopPixel);
        }

    }
}
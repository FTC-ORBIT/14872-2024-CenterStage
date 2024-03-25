package org.firstinspires.ftc.teamcode.Autonomous;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.AprilTagDetect;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.YellowPixelPosEnum;
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

@Autonomous(name = "Red Far Far Wall")
@Config
public class RedFarFromTheBoard extends  LinearOpMode{
    public static double maxVeloDrop = 6.5;
    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);
    public static double centerConeX = 29.7;
    public static double delay = 5000;
    public static double parkingY = -83;
    public static double parkingX = 46.19;
    public static double boardY = -85;
    public static double rightConeX = 33;

    public static double rightConeY = -3;


    double prepareToPropY = -1;

    public static double rightConeAngle =5.05;
    public static double leftConeX = 22.5;

    public static double leftConeY = 9;
    public static double centerAfterConeX = 23;
    public static double centerAfterConeY = 18;
    public static double centerGateX= 49.07;

    public static double centerGateSecondx = 55.07;
    public static double centerGateY = 16.41;
    public static double afterGateX = 55;
    public static double afterGateY = -70.345;
    public static double boardPos1 = 37;
    public static double boardPos2 = 34.7;
    public static double boardPos3HitRight = 32.5;
    public static double boardPos3MissLeft = 32;
    public static double boardPos4MissRight = 28.7;
    public static double boardPos4HitLeft = 28.4;
    public static double boardPos5 = 27;
    public static double boardPos6 = 22.5;
    // TODO X6 = 22.5
    // TODO X5 = 27
    // TODO X4 = 29.2
    // TODO X3 = 32
    // TODO X2 = 34.7
    // TODO X1 = 37
    public static double leftAfterPropX = 16;
    public static double leftAfterPropY = -3.5;
    public static double leftBeforeGateX =48;
    public static double rightAfterPropY = 3;
    public static double rightBeforeGateX = 52.8;
    public static double markerY = -77;
    public static double markerX = 30.0;

    public static double rightEndTangent = -30;
    public static ElevatorStates state = ElevatorStates.AUTO;
    private VisionPortal portal;
    private RedPropThresholdFar redPropThresholdFar = new RedPropThresholdFar();
    private AprilTagDetect aprilTag;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException{

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

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

        redPropThresholdFar.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag.atPrcsr)
                .addProcessor(redPropThresholdFar)
                .build();

        AprilTagDetect.setManualExposure(4,30, portal, this);
        portal.setProcessorEnabled(aprilTag.atPrcsr,false);


        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerConeX - 4, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerAfterConeX, centerAfterConeY ,startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerGateX , centerGateY ,startPose.getHeading()))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(centerGateSecondx, centerGateY , Math.toRadians(-90)))
                .waitSeconds(2.5)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(markerX, markerY,Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos4MissRight, boardY , Math.toRadians(-90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(0.3)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos4HitLeft, markerY,Math.toRadians(-90)))

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
                .lineToLinearHeading(new Pose2d(boardPos3HitRight, boardY , Math.toRadians(-90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(0.3)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos3HitRight, markerY,Math.toRadians(-90)))

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
                .lineToLinearHeading(new Pose2d(boardPos3MissLeft, boardY , Math.toRadians(-90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(1)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos3MissLeft, markerY,Math.toRadians(-90)))

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
                .lineToLinearHeading(new Pose2d(boardPos4MissRight, boardY , Math.toRadians(-90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(1)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos4MissRight, markerY,Math.toRadians(-90)))

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
                .lineToLinearHeading(new Pose2d(boardPos3HitRight, boardY , Math.toRadians(-90)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(0.3)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos3HitRight, markerY,Math.toRadians(-90)))

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
                .splineToLinearHeading(new Pose2d(rightConeX, prepareToPropY, Math.toRadians(-90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(rightConeX, rightConeY, Math.toRadians(-90)), Math.toRadians(-30))
                .lineToLinearHeading(new Pose2d(rightConeX, rightAfterPropY, rightConeAngle))
                .lineToLinearHeading(new Pose2d(rightBeforeGateX, rightAfterPropY , rightConeAngle))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(boardPos5, markerY,Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos6, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos6, markerY , Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos5, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos5, markerY , Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos5, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos5, markerY , Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos6, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos6, markerY , Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(markerX, markerY,Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos2, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(2)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos1,markerY,Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos1, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(2)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos2,markerY,Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos2, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(2)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos2,markerY,Math.toRadians(-90)))
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
                .lineToLinearHeading(new Pose2d(boardPos1, boardY , Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(2)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos1,markerY,Math.toRadians(-90)))
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
        while (!redPropThresholdFar.completedPropPos){
            telemetry.addLine("waiting for camera to start");
            telemetry.update();
            sleep(10);
        }
        if (!isStopRequested()) {
//            redPropThresholdFar.initYellowPixel();
            redPropThresholdFar.EnumGetPropPos();
            portal.stopStreaming();
            portal.setProcessorEnabled(aprilTag.atPrcsr, true);
            portal.setProcessorEnabled(redPropThresholdFar, false);

            switch (redPropThresholdFar.sampledPropPos) {
                case LEFT:
                    drive.followTrajectorySequence(leftCone);
                    getYellowPixelfromAprilTag();

                    while(!isStopRequested()) {
                        telemetry.addData("prop pos:", redPropThresholdFar.EnumGetPropPos());
                        telemetry.addData("yellow pixel:", redPropThresholdFar.sampledYellowPixelPos);
                        if (redPropThresholdFar.biggest != null) {
                            telemetry.addData("yellowThreshold", redPropThresholdFar.biggest.averagedBox);
                        }
                        telemetry.addData("wantedID", aprilTag.wantedID);
                        telemetry.addData("Detect attempts", aprilTag.count);
                        telemetry.update();
                        sleep(2000);
                        getYellowPixelfromAprilTag();

                    }

//                    if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITLEFT){
//                        drive.followTrajectorySequence(leftConeHitL);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITRIGHT){
//                        drive.followTrajectorySequence(leftConeHitR);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSRIGHT){
//                        drive.followTrajectorySequence(leftConeMissR);
//                    }else {
//                        drive.followTrajectorySequence(leftConeNopPixel);
//                    }
                    telemetry.addLine("left");
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
                    telemetry.addLine("center");

                    break;
                case RIGHT:
                    drive.followTrajectorySequence(rightCone);
                    getYellowPixelfromAprilTag();
//                    if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITLEFT){
//                        drive.followTrajectorySequence(rightConeHitL);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITRIGHT){
//                        drive.followTrajectorySequence(rightConeHitR);
//                    }else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSLEFT){
//                        drive.followTrajectorySequence(rightConeMissL);
//                    }else {
//                        drive.followTrajectorySequence(rightConeNopPixel);
//                    }
                    telemetry.addLine("right");
                    break;
                case NONE:
                    drive.followTrajectorySequence(centerCone);
                    getYellowPixelfromAprilTag();
//                    if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITLEFT) {
//                        drive.followTrajectorySequence(centerConeHitL);
//                    } else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITRIGHT){
//                        drive.followTrajectorySequence(centerConeHitR);
//                    } else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSLEFT){
//                        drive.followTrajectorySequence(centerConeMissL);
//                    } else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSRIGHT){
//                        drive.followTrajectorySequence(centerConeMissR);
//                    } else {
//                        drive.followTrajectorySequence(centerConeNopPixel);
//                    }
                    telemetry.addLine("Doesn't see prop");
                    break;
            }
            while (!isStopRequested()) {
                telemetry.addData("prop pos:", redPropThresholdFar.EnumGetPropPos());
                telemetry.addData("yellow pixel:", redPropThresholdFar.sampledYellowPixelPos);
                if(redPropThresholdFar.biggest != null) {
                    telemetry.addData("biggest:", redPropThresholdFar.biggest.averagedBox);
                }
                telemetry.addData("wantedID", aprilTag.wantedID);
                telemetry.addData("Detect attempts", aprilTag.count);
                telemetry.update();

                sleep(1000);
                redPropThresholdFar.test(gamepad1, telemetry);
            }
        }
    }

    public void getYellowPixelfromAprilTag(){
        portal.resumeStreaming();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING){
            sleep(5);
        }
        aprilTag.getAprilTagCords(redPropThresholdFar.sampledPropPos,
                                  redPropThresholdFar.AllianceColor);
        redPropThresholdFar.initYellowPixelAT(aprilTag.aprilTagCords);
        AprilTagDetect.setDfltExposure(portal,this);
        portal.setProcessorEnabled(redPropThresholdFar, true);
        sleep(100);
        redPropThresholdFar.getYellowPixelPos();
    }

    public void chooseDropTraj(TrajectorySequence ConeHitL,
                               TrajectorySequence ConeHitR,
                               TrajectorySequence ConeMissL,
                               TrajectorySequence ConeMissR,
                               TrajectorySequence ConeNopPixel){
        if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITLEFT) {
            drive.followTrajectorySequence(ConeHitL);
        } else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.HITRIGHT){
            drive.followTrajectorySequence(ConeHitR);
        } else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSLEFT){
            drive.followTrajectorySequence(ConeMissL);
        } else if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.MISSRIGHT){
            drive.followTrajectorySequence(ConeMissR);
        } else {
            drive.followTrajectorySequence(ConeNopPixel);
        }

    }
}
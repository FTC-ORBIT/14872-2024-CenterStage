package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThresholdClose;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.PlaneState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Close Close Wall")
@Config
public class RedCloseToTheBoard extends LinearOpMode {

    public static double maxVeloDrop = 6;

    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);

    public static TrajectoryVelocityConstraint maxVelConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint maxAccConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    public static double driveToConeX = 29.5;
    public static double goToParkingY = -39;
    ElapsedTime time = new ElapsedTime();
    public static double goToParkingX = 6;

    public static double delay = 1;
    public static double rightConeX = 22.5;
    public static double rightConeY = -8;

    public static double endTangent = 120;
    public static double leftConeX = 30;
    public static double leftConeY = 0.5;
    public static double prepareToPixelDropX = 22;
    public static double prepareToPixelDropY = 1.4;
    public static double boardPosY = -34;
    public static double boardPos12X = 35.5;
    public static double boardPos34X = 31;
    public static double boardPos56X = 18;
    public static double rightAfterConeX = 1.0;
    public static double rightAfterConeY=-18.73;
    public static double rightPreBoardY = -23.0;


    public static Pose2d endPose;

    private VisionPortal portal;
    private final RedPropThresholdClose redPropThresholdClose = new RedPropThresholdClose();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        redPropThresholdClose.initProp();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(redPropThresholdClose)
                .build();

        Elevator.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Outtake.init(hardwareMap);

        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(driveToConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(prepareToPixelDropX, prepareToPixelDropY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY + 10, Math.toRadians(-90)))
                .addTemporalMarker( ()-> {
                        Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                        Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(delay)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY + 8, Math.toRadians(-90)))
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
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeX, rightConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(rightAfterConeX , rightAfterConeY , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(boardPos56X, rightPreBoardY , startPose.getHeading()))
                .turn(Math.toRadians(-80))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos56X, boardPosY , Math.toRadians(-80)))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .lineToLinearHeading(new Pose2d(boardPos56X, rightPreBoardY , Math.toRadians(-85)))
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
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(leftConeX, leftConeY, Math.toRadians(90)), Math.toRadians(endTangent))
                .lineToLinearHeading(new Pose2d(prepareToPixelDropX, startPose.getY() - 10, Math.toRadians(90)))
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY + 6, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY, Math.toRadians(-90)))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(delay)
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY + 8, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.CLOSED);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateTeleop(FourbarState.REVERSE);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.INTAKE, telemetry);
                })
                .build();


        waitForStart();

        if (!isStopRequested()) {
            time.reset();
            time.startTime();
            telemetry.update();
            switch (redPropThresholdClose.EnumGetPropPos()) {
                case LEFT:
                    drive.followTrajectorySequence(leftCone);
                    endPose = leftCone.end();
                    telemetry.addLine("left");
                    break;
                case CENTER:
                    drive.followTrajectorySequence(centerCone);
                    endPose = centerCone.end();
                    telemetry.addLine("center");
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(rightCone);
                    endPose = rightCone.end();
                    telemetry.addLine("right");
                    break;
                case NONE:
                    drive.followTrajectorySequence(centerCone);
                    endPose = centerCone.end();
                    telemetry.addLine("none");
                    break;
            }
            TrajectorySequence parkCloseTheWall = drive.trajectorySequenceBuilder(endPose)
                    .lineToLinearHeading(new Pose2d(goToParkingX-3, goToParkingY + 5, Math.toRadians (-90)))
                    .turn(Math.toRadians(90))
                            .build();

            drive.followTrajectorySequence(parkCloseTheWall);
            telemetry.update();
        }
    }
}

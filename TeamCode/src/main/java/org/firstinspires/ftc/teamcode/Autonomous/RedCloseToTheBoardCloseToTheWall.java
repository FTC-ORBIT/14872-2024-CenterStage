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
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThresholdClose;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "RedCloseToTheBoardCloseToTheWall")
@Config
public class RedCloseToTheBoardCloseToTheWall extends LinearOpMode {

    public static double maxVeloDrop = 7;

    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);
    public static TrajectoryVelocityConstraint velConstraintLeft = SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintLeft = SampleMecanumDrive.getAccelerationConstraint(15);
    public static double driveToConeX = 31;
    public static double goToParkingY = -38;
    public static double delay = 1;
    public static double rightConeX = 22.5;
    public static double rightConeY = -8;
    public static double leftDriveX = 22.02;
    public static double leftConeX = 30;
    public static double leftConeY = 3.2;
    public static double prepareToPixelDropX = 25;
    public static double prepareToPixelDropY = 1.4;
    public static double boardPosY = -33;
    public static double boardPos12X = 35.5;
    public static double boardPos34X = 28.5;
    public static double boardPos56X = 20.8;

    private VisionPortal portal;
    private final RedPropThresholdClose redPropThresholdClose = new RedPropThresholdClose();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        redPropThresholdClose.initProp();

        Elevator.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Outtake.init(hardwareMap);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(redPropThresholdClose)
                .build();

        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(driveToConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(prepareToPixelDropX, prepareToPixelDropY, startPose.getHeading()))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY + 3, Math.toRadians(startPose.getHeading() - 90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY, Math.toRadians(startPose.getHeading() - 90)))
                .waitSeconds(delay)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.OUT);
                })
                .waitSeconds(delay)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY + 8, Math.toRadians(startPose.getHeading() - 90)))
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
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, Math.toRadians (startPose.getHeading() - 90)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeX, rightConeY, startPose.getHeading()))
                .lineToLinearHeading(startPose)
                                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, startPose.getHeading()))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(velConstraintLeft, accConstraintLeft)
                .lineToLinearHeading(new Pose2d(leftDriveX  , startPose.getY() , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftConeX, leftConeY, Math.toRadians(90)))
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(prepareToPixelDropX, startPose.getY() - 10, Math.toRadians(startPose.getHeading() + 90)))
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY + 3, Math.toRadians(startPose.getHeading() - 90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY, Math.toRadians(startPose.getHeading() - 90)))
                .waitSeconds(delay)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.OUT);
                })
                .waitSeconds(delay)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY + 8, Math.toRadians(startPose.getHeading() - 90)))
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
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, goToParkingY, Math.toRadians (startPose.getHeading() - 90)))
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            telemetry.update();
            switch (redPropThresholdClose.EnumGetPropPos()) {
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
                    drive.followTrajectorySequence(centerCone);
                    telemetry.addLine("none");
                    break;
            }
            telemetry.update();
        }
    }
}
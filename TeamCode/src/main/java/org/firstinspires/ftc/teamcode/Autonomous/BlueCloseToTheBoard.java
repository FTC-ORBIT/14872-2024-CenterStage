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
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThresholdClose;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Close Close Wall")
@Config
public class BlueCloseToTheBoard extends LinearOpMode {

    public static double maxVeloDrop = 10;

    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);
    public static TrajectoryVelocityConstraint velConstraintLeft = SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintLeft = SampleMecanumDrive.getAccelerationConstraint(15);
    public static double driveToConeX = 29;
    public static double goToParkingY = 27;
    public static double prepareToPropY = -3;
    public static double delay = 1;
    public static double rightConeX = 31;
    public static double rightConeY = -4.5;
    public static double leftConeX = 22;
    public static double leftConeY = 8;
    public static double prepareToPixelDropX = 25;
    public static double prepareToPixelDropY = -1.4;
    public static double boardPosY = 32;
    public static double boardPos12X = 35.8;
    public static double boardPos34X = 28;
    public static double boardPos56X = 20.2;
    public static double rightAfterConeX = 1.0;
    public static double rightAfterConeY=18.73;
    public static double rightPreBoardY = 19;
    public static double beforePixelBoardCenterY = 21.84;

    private VisionPortal portal;
    private final BluePropThresholdClose bluePropThresholdClose = new BluePropThresholdClose();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        bluePropThresholdClose.initProp();

        Elevator.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Outtake.init(hardwareMap);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdClose)
                .build();

        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(driveToConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(driveToConeX - 3 , startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(prepareToPixelDropX, prepareToPixelDropY, startPose.getHeading()))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(boardPos34X, beforePixelBoardCenterY, Math.toRadians(startPose.getHeading() + 90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY, Math.toRadians(startPose.getHeading() + 90)))
                .waitSeconds(delay)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .waitSeconds(delay)
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos34X, boardPosY - 8, Math.toRadians(startPose.getHeading() + 90)))
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
                .lineToLinearHeading(new Pose2d(startPose.getX() , goToParkingY, Math.toRadians (startPose.getHeading() + 90)))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(rightConeX, prepareToPropY, Math.toRadians(-90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(rightConeX, rightConeY, Math.toRadians(-90)), Math.toRadians(-30))
                .lineToLinearHeading(new Pose2d(prepareToPixelDropX, startPose.getY() + 10, Math.toRadians(startPose.getHeading() - 90)))
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY - 3, Math.toRadians(startPose.getHeading() + 90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY, Math.toRadians(startPose.getHeading() + 90)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(boardPos12X, boardPosY - 8, Math.toRadians(startPose.getHeading() + 90)))
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
                .lineToLinearHeading(new Pose2d(startPose.getX()+3 , goToParkingY , Math.toRadians(startPose.getHeading() + 90)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(startPose.getX()  , goToParkingY, Math.toRadians (startPose.getHeading())))
                .build();

        TrajectorySequence leftCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftConeX , leftConeY , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(rightAfterConeX,rightAfterConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(boardPos56X, rightPreBoardY , startPose.getHeading()))
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(boardPos56X, boardPosY , Math.toRadians(90)))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .lineToLinearHeading(new Pose2d(boardPos56X, rightPreBoardY , Math.toRadians(90)))  .addTemporalMarker(() -> {
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
                .lineToLinearHeading(new Pose2d(startPose.getX()   , goToParkingY, startPose.getHeading()))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            telemetry.update();
            switch (bluePropThresholdClose.EnumGetPropPos()) {
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
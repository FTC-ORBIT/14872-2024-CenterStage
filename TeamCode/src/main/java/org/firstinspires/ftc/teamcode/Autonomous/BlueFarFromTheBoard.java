package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.RedCloseToTheBoard.endTangent;

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
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThresholdFar;
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

@Autonomous(name = "Blue Far Far Wall")
@Config
public class BlueFarFromTheBoard extends  LinearOpMode{
    public static double maxVeloDrop = 10;
    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);
    public static double centerConeX = 29;
    public static double delay = 5000;
    public static double parkingY = 88;
    public static double parkingX = 46.19;
    public static double boardY = 84;
    public static double rightDriveX = 27;
    public static double rightConeX = 24;

    public static double rightConeY = -8;

    public static double rightConeAngle =-5.05;
    public static double leftConeX = 30;

    public static double leftConeY = -0.5;
    public static double centerAfterConeX = 23;
    public static double centerAfterConeY = -18;
    public static double centerGateX= 49.07;
    public static double centerGateY = -16.41;
    public static double afterGateX = 53;
    public static double afterGateY = 70.345;
    public static double boardPos12 = 35;
    public static double boardPos34 = 27.9;
    public static double boardPos56 = 22.5;
    // TODO "MUST READ: in the BLUE autonomous the 1-6 is from right to left!!"
    // TODO X6 = 22.5
    // TODO X5 = 22.9
    // TODO X4 = 27.86
    // TODO X3 = 28.19
    // TODO X2 = 33.7
    // TODO X1 = 35
    public static double leftAfterPropX = 16;
    public static double leftAfterPropY = 3.5;
    public static double leftBeforeGateX =48;
    public static double rightAfterPropY = -3;
    public static double rightBeforeGateX = 52;
    public static double markerY = 75;

    private VisionPortal portal;
    private BluePropThresholdFar bluePropThresholdFar = new BluePropThresholdFar();

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Elevator.init(hardwareMap);
        Outtake.init(hardwareMap);
        Intake.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Plane.init(hardwareMap);

        bluePropThresholdFar.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdFar)
                .build();



        TrajectorySequence centerCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(centerConeX, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerConeX - 4, startPose.getY(), startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerAfterConeX, centerAfterConeY ,startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(centerGateX , centerGateY ,startPose.getHeading()))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(centerGateX +3, centerGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(boardPos34, markerY,Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos34 , boardY , Math.toRadians(90)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
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
                .turn(startPose.getHeading())
                .lineToLinearHeading(new Pose2d(parkingX, parkingY , startPose.getHeading()))
                .build();

        TrajectorySequence rightCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightConeX, rightConeY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(startPose.getX() + 3, leftConeY , startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftAfterPropX, leftAfterPropY, startPose.getHeading()))
                .lineToLinearHeading(new Pose2d(leftBeforeGateX, leftAfterPropY , startPose.getHeading()))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(leftBeforeGateX + 5, leftAfterPropY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(boardPos12, markerY,Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos12 , boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(boardPos12,markerY,Math.toRadians(90)))
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
                .splineToLinearHeading(new Pose2d(leftConeX, leftConeY, Math.toRadians(90)), Math.toRadians(endTangent))
                .lineToLinearHeading(new Pose2d(leftConeX,leftConeY - 2.5,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(rightConeX, rightAfterPropY, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(rightBeforeGateX, rightAfterPropY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(afterGateX, afterGateY , Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(boardPos56, markerY,Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .waitSeconds(1)
                .setConstraints(velConstraintDrop, accConstraintDrop)
                .lineToLinearHeading(new Pose2d(boardPos56 , boardY , Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    Outtake.operate(OuttakeState.TOWOUT);
                })
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(boardPos56, markerY , Math.toRadians(90)))
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
                .lineToLinearHeading(new Pose2d(parkingX, parkingY-2 , startPose.getHeading()))
                .build();

        TrajectorySequence elevatorCheck = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() ->{
                    Elevator.operateAutonomous(ElevatorStates.MIN, telemetry);
                })
                        .build();

        waitForStart();

        if (!isStopRequested()) {
            sleep((long) delay);
            switch (bluePropThresholdFar.EnumGetPropPos()) {
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
                    drive.followTrajectorySequence(centerCone);
                    telemetry.addLine("Doesn't see prop");
                    telemetry.update();
                    break;
            }

        }
    }
}


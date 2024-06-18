package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.AprilTagDetect;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.BluePropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@TeleOp(group = "Tests")
public class LocalizationTest extends LinearOpMode {
    private VisionPortal portal;
    private AprilTagDetect aprilTag;

    private BluePropThresholdFar bluePropThresholdFar = new BluePropThresholdFar();
    private boolean systemsOut = false;
    @Override
    public void runOpMode() throws InterruptedException {

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevator.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Outtake.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag.atPrcsr)
                .addProcessor(bluePropThresholdFar)
                .build();

        FtcDashboard.getInstance().startCameraStream(portal, 30);
        portal.setProcessorEnabled(aprilTag.atPrcsr,false);
        portal.setProcessorEnabled(bluePropThresholdFar,false);



        waitForStart();
        systemsOut = false;

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            if (gamepad1.a || systemsOut) {
                systemsOut = true;
                Elevator.operateAutonomous(ElevatorStates.AUTO, telemetry);
                Fourbar.operateAutonomous(FourbarState.MOVE);
                if (gamepad1.left_bumper) Outtake.operate(OuttakeState.TOWOUT);
              if (gamepad1.right_bumper) Outtake.operate(OuttakeState.CLOSED);
            }
             if (gamepad1.b) {
                systemsOut = false;
              Fourbar.operateAutonomous(FourbarState.REVERSE);
              sleep(1000);
              Elevator.operateAutonomous(ElevatorStates.INTAKE,telemetry);
              Outtake.operate(OuttakeState.CLOSED);
            }
if (gamepad1.dpad_up){
    portal.setProcessorEnabled(aprilTag.atPrcsr,true);

}
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            }

        }
    }


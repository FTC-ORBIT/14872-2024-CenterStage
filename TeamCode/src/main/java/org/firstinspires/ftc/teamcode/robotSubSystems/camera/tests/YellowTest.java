package org.firstinspires.ftc.teamcode.robotSubSystems.camera.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.AprilTagDetect;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.BluePropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.RedPropThresholdFar;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.enums.PropPosEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Autonomous (name = "Yellow pixel detect",group = "Tests")
@Disabled
public class YellowTest extends LinearOpMode {
    private VisionPortal portal;
    private BluePropThresholdFar bluePropThresholdFar = new BluePropThresholdFar();
    private AprilTagDetect aprilTag;

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

        bluePropThresholdFar.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag.atPrcsr)
                .addProcessor(bluePropThresholdFar)
                .build();
        AprilTagDetect.setManualExposure(4, 30, portal, this);
//        portal.setProcessorEnabled(aprilTag.atPrcsr,false);

        FtcDashboard.getInstance().startCameraStream(portal, 30);

        waitForStart();
        while (!bluePropThresholdFar.completedPropPos){
            telemetry.addLine("waiting for camera to start");
            telemetry.update();
            sleep(10);
        }
        if (!isStopRequested()) {
            bluePropThresholdFar.EnumGetPropPos();
            RedPropThresholdFar.sampledPropPos = PropPosEnum.LEFT;
            portal.stopStreaming();
            portal.setProcessorEnabled(aprilTag.atPrcsr, true);
            portal.setProcessorEnabled(bluePropThresholdFar, false);

            getYellowPixelfromAprilTag(); // here you get sampledYellowPixelPos "at start" (which is the first yellowpixelpos)
                while (!isStopRequested()) {
                    print_tele(1000, true);
                    telemetry.addData("prop pos:", bluePropThresholdFar.EnumGetPropPos());
                    telemetry.addData("yellow pixel at start:", bluePropThresholdFar.sampledYellowPixelPos);
                    telemetry.addData("yellow pixel right now:", bluePropThresholdFar.yellowPixelPos);
                    if (bluePropThresholdFar.biggest != null) {
                        telemetry.addData("biggest", bluePropThresholdFar.biggest.averagedBox);
                    }
                    telemetry.addData("wantedID", aprilTag.wantedID);
                    telemetry.addData("Detect attempts", aprilTag.count);
                    telemetry.update();
                    sleep(2000);

//                telemetry.addLine("yay debug");
//                telemetry.update();
//                sleep(500);
//                getYellowPixelfromAprilTag();
                bluePropThresholdFar.getYellowPixelPos();
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


        }
    }

    public void getYellowPixelfromAprilTag() {
        portal.resumeStreaming();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(5);
        }

        aprilTag.getAprilTagCords(bluePropThresholdFar.sampledPropPos,
                bluePropThresholdFar.AllianceColor);
        bluePropThresholdFar.initYellowPixelAT(aprilTag.aprilTagCords);
        AprilTagDetect.setDfltExposure(portal, this);
        portal.setProcessorEnabled(bluePropThresholdFar, true);
// TODO check the sleep time here (might be not enough time):
//        sleep(100);
        bluePropThresholdFar.getYellowPixelPos();
    }
    public void print_tele (long millisec, boolean test){
        telemetry.addLine("print tele");
        telemetry.addData("prop pos", bluePropThresholdFar.EnumGetPropPos());
        telemetry.addData("yellow pixel at start", bluePropThresholdFar.sampledYellowPixelPos);
        telemetry.addData("yellow pixel right now", bluePropThresholdFar.yellowPixelPos);
        if(bluePropThresholdFar.biggest != null) {
            telemetry.addData("biggest", bluePropThresholdFar.biggest.averagedBox);
        }
        telemetry.addData("wantedID", aprilTag.wantedID);
        telemetry.addData("Detect attempts", aprilTag.count);
        telemetry.addData("boxs width",AprilTagDetect.rectWidth);
        telemetry.update();

//        if (redPropThresholdFar.sampledYellowPixelPos == YellowPixelPosEnum.NOPIXEL){
//            while(!isStopRequested()) {
//                sleep(100);
//            }
//        }
        sleep(millisec);
        if (test) {
            bluePropThresholdFar.test(gamepad1, telemetry);
        }
    }
}



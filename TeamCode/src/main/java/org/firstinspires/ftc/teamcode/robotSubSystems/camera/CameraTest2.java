package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.List;

@Autonomous  (name ="Vision Test Red")
@Config

    public class CameraTest2 extends LinearOpMode {

        private VisionPortal portal;
        private AprilTagProcessor aprilTag;
        private Point aprilTagCenter;
        private RedPropThresholdFar redPropThreshold = new RedPropThresholdFar();


        private Delay delayCV = new Delay(3);
    public Gamepad lastGamepad = new Gamepad();
    // Create the AprilTag processor.

    public static double propPos = 0;



    @SuppressLint("DefaultLocale")
    @Override
        public void runOpMode() throws InterruptedException {
            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.

                    .build();

            ElapsedTime robotTime = new ElapsedTime();
            robotTime.reset();
            redPropThreshold.initProp();
            switch ((int) propPos) {
                case 0:
                    redPropThreshold.PropPos = PropPosEnum.LEFT;
                break;
                case 1:
                    redPropThreshold.PropPos = PropPosEnum.CENTER;
                    break;
                case 2:
                    redPropThreshold.PropPos = PropPosEnum.RIGHT;
                    break;
            }
    //        redPropThreshold.initYellowPixel();


            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                    .setCameraResolution(new Size(640, 480))
//                    .setCameraResolution(new Size(640, 360))
//                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .addProcessor(redPropThreshold)
                    .build();

            portal.setProcessorEnabled(redPropThreshold, false);

//            delayCV.startAction((float) robotTime.seconds());
//
//            while (!delayCV.isDelayPassed()) {
//                GlobalData.currentTime = (float) robotTime.seconds();
//                telemetry.addLine("waiting...");
//                telemetry.addData("time(robottime)", robotTime);
//                telemetry.addData("time (current time)", GlobalData.currentTime);
//
//                telemetry.update();
//            }
//            redPropThreshold.initYellowPixel();

            boolean toggle = false;
            boolean sttogle = false;
            List<AprilTagDetection> currentDetections = null;


            while (!isStopRequested()) {
                if(gamepad2.a && !lastGamepad.a) {
                    toggle = !toggle;
                    portal.setProcessorEnabled(aprilTag, toggle);
                }
                if(gamepad2.b && !lastGamepad.b) {
                    if(!sttogle) {
                        sttogle = true;
                        portal.resumeStreaming();
                    }
                    else {
                        sttogle = false;
                        portal.stopStreaming();
                    }
                }

                lastGamepad.copy(gamepad2);

                telemetry.addData("AprilTags processor enabled:", portal.getProcessorEnabled(aprilTag));
                telemetry.addData("PropThreshold processor enabled:", portal.getProcessorEnabled(redPropThreshold));
                telemetry.addData("the pixel is in:", redPropThreshold.getYellowPixelPos());
                telemetry.addData("Prop Position", redPropThreshold.EnumGetPropPos());
                telemetry.addLine("");


                while(aprilTagCenter == null){
                    currentDetections = aprilTag.getDetections();

                    telemetry.addData("# AprilTags Detected", currentDetections.size());
                    for(AprilTagDetection i : currentDetections) {
                        if (i.metadata != null) {
                            if (i.id == 5) { //TODO set id based on propcolor and position
                                aprilTagCenter = i.center;
                            }
                        }
                    }
                    telemetry.update();
                    sleep(5*1000);
                }

                redPropThreshold.aprilTagCenter = aprilTagCenter;
                sleep(500);
                redPropThreshold.initYellowPixelAT();
                portal.setProcessorEnabled(redPropThreshold, true);
                telemetry.addLine("Yellow Pixel Detection Processor is DISABLED - See PropThreshod Code !!!!!");
                for (ElementDetectBox eBox : redPropThreshold.yellowBoxesHash) {
                    telemetry.addLine(String.format("\n==== %s %s", eBox.place.toString(), eBox.elementBox.toString()));
                }
                currentDetections = aprilTag.getDetections();

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                        telemetry.addLine(String.format("corner[0] (%6.1f %6.1f)  Center (%6.1f %6.1f)  ()", detection.corners[0].x, detection.corners[0].y, detection.center.x, detection.center.y));
                        if (detection.id == 5) { //TODO set id based on propcolor and position
                            aprilTagCenter = detection.center;
                        }
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }   // end for() loop

                // Add "key" information to telemetry
                telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");
                telemetry.addLine("");


                telemetry.addData("Left Box: ", redPropThreshold.leftBox);
                telemetry.addData("Middle Box: ", redPropThreshold.middleBox);
                telemetry.addData("Right Box:", redPropThreshold.rightBox);
                telemetry.addData("Averaged Left Box:", redPropThreshold.averagedLeftBox);
                telemetry.addData("Averaged Middle Box:", redPropThreshold.averagedMiddleBox);
                telemetry.addData("Averaged Right Box:", redPropThreshold.averagedRightBox);


                redPropThreshold.test(gamepad2, telemetry);
                telemetry.update();
            }
        }
    }

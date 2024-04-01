package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.core.Point;

import java.util.List;
import java.util.concurrent.TimeUnit;

//public class AprilTagDetect extends AprilTagProcessorImpl {
public class AprilTagDetect {

    public static AprilTagProcessor atPrcsr;
    public static Point aprilTagCords;
    public static int wantedID = 2;
    static List<AprilTagDetection> currentDetections = null;
    public static int count = 0;
    public static int tries = 200;


//    public AprilTagDetect(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, AprilTagProcessor.TagFamily tagFamily, int threads, boolean suppressCalibrationWarnings) {
//      super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube,drawOutline, drawTagID, tagFamily, threads, suppressCalibrationWarnings);
//    }

    public AprilTagDetect(){
    }

    protected static int getIDfromPosAndColor(PropPosEnum pos, PropColorEnum allianceColor) {

        switch (pos) {
            case LEFT:
                wantedID = 1;
                break;
            case CENTER:
            default:
                wantedID = 2;
                break;
            case RIGHT:
                wantedID = 3;
                break;
        }

        if (allianceColor == PropColorEnum.RED) {
            wantedID += 3;
        }
        return wantedID;
    }

    public static Point getAprilTagCords(PropPosEnum pos, PropColorEnum allianceColor) {
        getIDfromPosAndColor(pos, allianceColor);
        count = 0;
        aprilTagCords = null;

        if (atPrcsr != null) {
            while (count < tries && aprilTagCords == null) {
                currentDetections = atPrcsr.getDetections();

                count++;
//                telemetry.addData("# AprilTags Detected", currentDetections.size());
                for (AprilTagDetection dtct : currentDetections) {
                    if (dtct.metadata != null) {
                        if (dtct.id == wantedID) {
                            // dtct.center is the center in x y cords and not center as a pos
                            aprilTagCords = dtct.center;
                        }
                    }
                }
                sleep(5);
            }
            return aprilTagCords;
        } else {
            return new Point(415, 300);
        }
    }

    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    public static boolean    setManualExposure(int exposureMS, int gain, VisionPortal camera, LinearOpMode opMd) {
        // Ensure Vision Portal has been setup.
        if (camera == null) {
            return false;
        }

        // Wait for the camera to be open
        if (camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMd.telemetry.addData("Camera", "Waiting");
            opMd.telemetry.update();
            while (!opMd.isStopRequested() && (camera.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            opMd.telemetry.addData("Camera", "Ready");
            opMd.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMd.isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = camera.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = camera.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    public static boolean    setDfltExposure(VisionPortal camera, LinearOpMode opMd) {
        // Ensure Vision Portal has been setup.
        if (camera == null) {
            return false;
        }

        // Wait for the camera to be open
        if (camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMd.telemetry.addData("Camera", "Waiting");
            opMd.telemetry.update();
            while (!opMd.isStopRequested() && (camera.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            opMd.telemetry.addData("Camera", "Ready");
            opMd.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMd.isStopRequested()) {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = camera.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.AperturePriority) {
                exposureControl.setMode(ExposureControl.Mode.AperturePriority);
                sleep(50);
            }
            return (true);
        } else {
            return (false);
        }
    }
}


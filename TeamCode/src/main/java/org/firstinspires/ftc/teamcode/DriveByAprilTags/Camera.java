package org.firstinspires.ftc.teamcode.DriveByAprilTags;

import static org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain.motors;

import android.widget.Switch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
@Config
public class Camera {
    public static boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    static double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    static double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    static double  turn            = 0;// Desired turning power/speed (-1 to +1)
    public static final double FINAL_DESIRED_DISTANCE = 12;

    public static double DESIRED_DISTANCE = FINAL_DESIRED_DISTANCE; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public  static double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE= 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static final int  DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static VisionPortal visionPortal;               // Used to manage the video source.
    private static AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public static AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag
    public static double driveTank;
    public static double turnTank;
    public static boolean skippingTagTelemetry = false;
    public static double  rangeError;
    public static double  headingError;
    public static double  yawError;
    public static boolean ElevatorStateAprilTagsSwitch = false;
    public static boolean FourBarStateAprilTagsSwitch = false;
    public static boolean OuttakeStateAprilTagsSwitch = false;
    public static boolean breakAutoDrive = false;
    public static boolean resetSystems = false;
    public static Delay Outtakedelay = new Delay(0.6f);
    public static Delay autoBreakDelay = new Delay(0.8f);
   public static CameraEnum currentState;
   public static boolean driveClose = true;


    public static void initAprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setLensIntrinsics(458.066, 457.626, 337.176, 251.805)
                .build();

        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
        if (USE_WEBCAM)  setManualExposure(8 , 250, telemetry);  // Use low exposure time to reduce motion blur
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private static void    setManualExposure(int exposureMS, int gain, Telemetry telemetry) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public static void update(){

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    skippingTagTelemetry = false;
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    desiredTag = detection;
                    skippingTagTelemetry = true;
                    targetFound = false;
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                skippingTagTelemetry = false;
                targetFound = false;
            }
        }
        FtcDashboard.getInstance().startCameraStream(visionPortal,20);
    }


    public static void getAprilTagDetectionOmni(){
        if(targetFound){
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
              rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
              headingError    = desiredTag.ftcPose.bearing;
              yawError        = desiredTag.ftcPose.yaw;
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
          moveRobot(drive,strafe,turn);

        }else {
         motors[0].setPower(0);
         motors[1].setPower(0);
         motors[2].setPower(0);
         motors[3].setPower(0);
        }
        if (rangeError < 0.78 && headingError < 1.5 && yawError < 0.78 && !resetSystems) {
            currentState = CameraEnum.OPENSYSTEMS;
        }else {
            breakAutoDrive = false;
        }
        switch (currentState){
            case OPENSYSTEMS:
                ElevatorStateAprilTagsSwitch = true;
                FourBarStateAprilTagsSwitch = true;
                currentState = CameraEnum.DRIVEFORWARD;
                break;
            case DRIVEFORWARD:
                if (driveClose) {
                    DESIRED_DISTANCE = 7;
                }
                if (rangeError < 0.78 && headingError < 1.5 && yawError < 0.78){
                    currentState = CameraEnum.DROP;
                }
                break;
            case DROP:
                Outtakedelay.startAction(GlobalData.currentTime);
                OuttakeStateAprilTagsSwitch = true;
                if (Outtakedelay.isDelayPassed()){
                    currentState = CameraEnum.DRIVEBACK;
                }
                break;
            case DRIVEBACK:
                driveClose = false;
                DESIRED_DISTANCE = FINAL_DESIRED_DISTANCE;
                if (rangeError < 0.78 && headingError < 1.5 && yawError < 0.78) {
                    currentState = CameraEnum.CLOSESYSTEMS;
                }
                    break;
            case CLOSESYSTEMS:
                ElevatorStateAprilTagsSwitch = false;
                FourBarStateAprilTagsSwitch = false;
                OuttakeStateAprilTagsSwitch = false;
                DESIRED_DISTANCE = FINAL_DESIRED_DISTANCE;
                currentState = CameraEnum.BREAKAUTODRIVE;
                break;
            case BREAKAUTODRIVE:
                autoBreakDelay.startAction(GlobalData.currentTime);
                if (autoBreakDelay.isDelayPassed()) {
                    breakAutoDrive = true;
                }
                break;
        }
    }
    public static void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motors[0].setPower(leftFrontPower);
        motors[1].setPower(rightFrontPower);
        motors[2].setPower(leftBackPower);
        motors[3].setPower(rightBackPower);
    }
}


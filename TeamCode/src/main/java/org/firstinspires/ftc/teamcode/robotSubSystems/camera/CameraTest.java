package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;

@Autonomous(name="Vision Test")
public class CameraTest extends LinearOpMode {
    public long captureTimeNanos;
    private VisionPortal portal;
    private BluePropThreshold bluePropThreshold = new BluePropThreshold();
    public Mat frame;

    @Override
    public void runOpMode() throws InterruptedException {

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .build();


        while (!isStopRequested()) {
            telemetry.addData("Prop Position: ", bluePropThreshold.getPropPosition());
            telemetry.addData("left box: " , bluePropThreshold.leftBox);
            telemetry.addData("right box: " , bluePropThreshold.rightBox);
            telemetry.update();
        }//Will output prop position on Driver Station Console



    }
}

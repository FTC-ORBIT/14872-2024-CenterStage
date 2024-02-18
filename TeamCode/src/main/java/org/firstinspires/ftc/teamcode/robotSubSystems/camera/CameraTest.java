package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Test")
public class CameraTest extends LinearOpMode {
    private VisionPortal portal;
    private BluePropThreshold bluePropThreshold = new BluePropThreshold();

    @Override
    public void runOpMode() throws InterruptedException {

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
//                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .build();


        while (!isStopRequested()) {
            telemetry.addData("Prop Position:", bluePropThreshold.getPropPosition());
            telemetry.addData("left box:" , bluePropThreshold.blueLeftBox);
            telemetry.addData("right box:" , bluePropThreshold.blueMiddleBox);
            telemetry.addData("Averaged Left Box:", bluePropThreshold.averagedBlueLeftBox);
            telemetry.addData("Averaged Right Box:", bluePropThreshold.averagedRedMiddleBox);
            telemetry.update();
        }//Will output prop position on Driver Station Console



    }
}

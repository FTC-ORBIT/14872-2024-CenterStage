package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(name="Vision Test")
public class CameraTest extends LinearOpMode {

    private VisionPortal portal;
    private RedPropThreshold redPropThreshold = new RedPropThreshold();

    @Override
    public void runOpMode() throws InterruptedException {

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .setCamera(BuiltinCameraDirection.BACK)
                .build();


        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.update();                        //Will output prop position on Driver Station Console
        }



    }
}

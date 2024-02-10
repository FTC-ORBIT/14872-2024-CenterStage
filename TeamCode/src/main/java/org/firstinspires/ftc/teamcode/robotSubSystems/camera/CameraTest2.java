package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Test2")
    public class CameraTest2 extends LinearOpMode {

        private VisionPortal portal;
        RedPropThreshold redPropThreshold = new RedPropThreshold();


    @Override
        public void runOpMode() throws InterruptedException {

            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                    .setCameraResolution(new Size(640, 480))
//                    .setCameraResolution(new Size(640, 360))
//                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(redPropThreshold)
                    .build();
            while (!isStopRequested()) {
                telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
                telemetry.addData("Left Box: ", redPropThreshold.leftBox);
                telemetry.addData("Right Box: ", redPropThreshold.middleBox);
                telemetry.addData("Averaged Left Box:", redPropThreshold.averagedLeftBox);
                telemetry.addData("Averaged Right Box:", redPropThreshold.averagedMiddleBox);
                telemetry.update();

            }

        }
    }

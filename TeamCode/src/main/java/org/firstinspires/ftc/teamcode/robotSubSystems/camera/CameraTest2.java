package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Test Red")
    public class CameraTest2 extends LinearOpMode {

        private VisionPortal portal;
        private BluePropThresholdClose redPropThreshold = new BluePropThresholdClose();


    @Override
        public void runOpMode() throws InterruptedException {
    redPropThreshold.initProp();
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                    .setCameraResolution(new Size(640, 480))
//                    .setCameraResolution(new Size(640, 360))
//                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(redPropThreshold)
                    .build();
            while (!isStopRequested()) {
                telemetry.addData("Prop Position", redPropThreshold.GetPropPosition());
                telemetry.addData("Left Box: ", redPropThreshold.leftBox);
                telemetry.addData("Middle Box: ", redPropThreshold.middleBox);
                telemetry.addData("Right Box:", redPropThreshold.rightBox);
                telemetry.addData("Averaged Left Box:", redPropThreshold.averagedLeftBox);
                telemetry.addData("Averaged Middle Box:", redPropThreshold.averagedMiddleBox);
                telemetry.addData("Averaged Right Box:", redPropThreshold.averagedRightBox);
                telemetry.update();

            }

        }
    }

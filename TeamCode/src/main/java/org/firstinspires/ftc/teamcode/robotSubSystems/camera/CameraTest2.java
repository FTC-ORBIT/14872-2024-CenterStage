package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous  (name ="Vision Test Red")
@Config

    public class CameraTest2 extends LinearOpMode {

        private VisionPortal portal;
        private RedPropThresholdFar redPropThreshold = new RedPropThresholdFar();


        private Delay delayCV = new Delay(3);
        public static double propPos = 0;



    @Override
        public void runOpMode() throws InterruptedException {
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
        redPropThreshold.initYellowPixel();


        portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                    .setCameraResolution(new Size(640, 480))
//                    .setCameraResolution(new Size(640, 360))
//                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(redPropThreshold)
                    .build();

    delayCV.startAction((float) robotTime.seconds());

    while (!delayCV.isDelayPassed()) {
        GlobalData.currentTime = (float) robotTime.seconds();
        telemetry.addLine("waiting...");
        telemetry.addData("time(robottime)", robotTime);
        telemetry.addData("time (current time)", GlobalData.currentTime);

        telemetry.update();
    }
//            redPropThreshold.initYellowPixel();


            while (!isStopRequested()) {
                redPropThreshold.test(gamepad1, telemetry);
                telemetry.addData("the pixel is in:", redPropThreshold.getYellowPixelPos());
//                telemetry.addData("Prop Position", redPropThreshold.GetPropPosition());
                telemetry.addData("Prop Position", redPropThreshold.EnumGetPropPos());
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

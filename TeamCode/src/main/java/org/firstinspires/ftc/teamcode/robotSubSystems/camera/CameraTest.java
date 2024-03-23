package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name="Vision Test Blue")
public class CameraTest extends LinearOpMode {
    private VisionPortal portal;
    private BluePropThresholdFar bluePropThreshold = new BluePropThresholdFar();

    @Override
    public void runOpMode() throws InterruptedException {
        bluePropThreshold.initProp();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThreshold)
                .build();

        bluePropThreshold.initYellowPixel();

        while (!isStopRequested()) {
            telemetry.addData("the pixel is in:", bluePropThreshold.getYellowPixelPos()) ;
//            telemetry.addData("the prop is in:", bluePropThreshold.GetPropPosition());
            telemetry.addData("the prop is in:", bluePropThreshold.EnumGetPropPos());
            telemetry.addData("left box:" , bluePropThreshold.leftBox);
            telemetry.addData("middle box:" , bluePropThreshold.middleBox);
            telemetry.addData("right box:", bluePropThreshold.rightBox);
            telemetry.addData("Averaged Left Box:", bluePropThreshold.averagedLeftBox);
            telemetry.addData("Averaged Middle Box:", bluePropThreshold.averagedMiddleBox);
            telemetry.addData("Averaged Right Box:", bluePropThreshold.averagedRightBox);


            bluePropThreshold.test(gamepad1, telemetry);
            telemetry.update();


        }//Will output prop position on Driver Station Console



    }
}

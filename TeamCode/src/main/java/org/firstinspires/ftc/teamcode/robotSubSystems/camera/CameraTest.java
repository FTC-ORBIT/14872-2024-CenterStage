package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Test Blue")
public class CameraTest extends LinearOpMode {
    private VisionPortal portal;
    private BluePropThresholdClose bluePropThresholdClose = new BluePropThresholdClose();

    @Override
    public void runOpMode() throws InterruptedException {
        bluePropThresholdClose.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdClose)
                .build();


        while (!isStopRequested()) {
            telemetry.addData("the prop is in:", bluePropThresholdClose.GetPropPosition());
            telemetry.addData("left box:" , bluePropThresholdClose.leftBox);
            telemetry.addData("middle box:" , bluePropThresholdClose.middleBox);
            telemetry.addData("right box:", bluePropThresholdClose.rightBox);
            telemetry.addData("Averaged Left Box:", bluePropThresholdClose.averagedLeftBox);
            telemetry.addData("Averaged Middle Box:", bluePropThresholdClose.averagedMiddleBox);
            telemetry.addData("Averaged Right Box:", bluePropThresholdClose.averagedRightBox);


            bluePropThresholdClose.test(gamepad1, telemetry);
            telemetry.update();


        }//Will output prop position on Driver Station Console



    }
}

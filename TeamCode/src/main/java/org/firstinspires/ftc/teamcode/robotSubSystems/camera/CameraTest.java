package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Test Blue")
public class CameraTest extends LinearOpMode {
    private VisionPortal portal;
    private RedPropThresholdClose redPropThresholdClose = new RedPropThresholdClose();

    @Override
    public void runOpMode() throws InterruptedException {
        redPropThresholdClose.initProp();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(redPropThresholdClose)
                .build();


        while (!isStopRequested()) {
            telemetry.addData("the prop is in:", redPropThresholdClose.GetPropPosition());
            telemetry.addData("left box:" , redPropThresholdClose.leftBox);
            telemetry.addData("middle box:" , redPropThresholdClose.middleBox);
            telemetry.addData("right box:", redPropThresholdClose.rightBox);
            telemetry.addData("Averaged Left Box:", redPropThresholdClose.averagedLeftBox);
            telemetry.addData("Averaged Middle Box:", redPropThresholdClose.averagedMiddleBox);
            telemetry.addData("Averaged Right Box:", redPropThresholdClose.averagedRightBox);
            telemetry.update();
        }//Will output prop position on Driver Station Console



    }
}

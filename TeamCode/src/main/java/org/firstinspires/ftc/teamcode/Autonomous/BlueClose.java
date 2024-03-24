package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThresholdClose;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous (name = "blue close")
public class BlueClose extends AutonomousGenaral{

    private VisionPortal portal;
    private final BluePropThresholdClose bluePropThresholdClose = new BluePropThresholdClose();
    int position = 1;

    boolean color = false;

    String parkingPos = "close";
    AutonomousSteps currentState;
    @Override
    public void runOpMode() throws InterruptedException {
        bluePropThresholdClose.initProp();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(bluePropThresholdClose)
                .build();

        init(hardwareMap);

        waitForStart();
if (isStopRequested()) return;

        currentState = AutonomousSteps.PROP;

        telemetry.update();
        switch (bluePropThresholdClose.EnumGetPropPos()) {
            case LEFT:
                position = 1;
                telemetry.addLine("left");
                break;
            case CENTER:
                position = 2;
                telemetry.addLine("center");
                break;
            case RIGHT:
                position = 3;
                telemetry.addLine("right");
                break;
            case NONE:
                position = 1;
                telemetry.addLine("none");
                break;
        }

        purplePixelDrop13(position);

        while (opModeIsActive() && !isStopRequested()){

            switch (currentState){
                case PROP:
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.PREPARETODROPPIXEL;
                        prepareToPixelDrop23(position, color);
                    }
                    telemetry.addData("prop", null);
                    break;
                case PREPARETODROPPIXEL:
                    telemetry.addData("PREPARETODROPPIXEL", null);
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.OPENSYSTEMS;
                        markerBoard(telemetry);
                    }
                    break;
                case OPENSYSTEMS:
                        if (!drive.isBusy()){
                            dropYellowPixel23(position, color);
                            currentState = AutonomousSteps.GOTOBOARD;
                        }

                    break;
                case GOTOBOARD:
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.DROPPIXEL;
                        time.reset();
                    }
                    break;
                case DROPPIXEL:
                    Outtake.operate(OuttakeState.TOWOUT);
                    if (time.seconds() > dropYellowPixelDelay){
                        currentState = AutonomousSteps.FARFROMTHEBOARD;
                        prepareToPixelDrop23(position, color);
                    }
                    break;
                case FARFROMTHEBOARD:
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.CLOSESYSTEMS;
                        time.reset();
                    }
                    break;
                case CLOSESYSTEMS:
                    Fourbar.operateAutonomous(FourbarState.REVERSE);
                    if (time.seconds() > elevatorClosingDelay) {
                        Elevator.operateAutonomous(ElevatorStates.INTAKE, telemetry);
                        if (Elevator.reachedHeight(Elevator.getPos())){
                            currentState = AutonomousSteps.GOTOPARKING;
                            parking(position, parkingPos);
                        }
                    }
                    break;
                case GOTOPARKING:
                  telemetry.addData("parked", null);
            }
            drive.update();
            telemetry.update();
        }


    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.BluePropThresholdClose;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (name = "Blue Close", group = "Blue")
public class BlueClose extends AutonomousGenaral{

    private VisionPortal portal;
    private final BluePropThresholdClose bluePropThresholdClose = new BluePropThresholdClose();
    ElapsedTime time = new ElapsedTime();
    int propPosition = 1;
    int robotPosition = 3;

    boolean color = false;

    String parkingPos = "close";
    AutonomousSteps currentState;

    ElevatorStates elevatorStates = ElevatorStates.INTAKE;

    FourbarState fourbarState = FourbarState.REVERSE;

    OuttakeState outtakeState = OuttakeState.CLOSED;

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
time.reset();

        currentState = AutonomousSteps.PROP;

        telemetry.update();
        switch (bluePropThresholdClose.EnumGetPropPos()) {
            case LEFT:
                propPosition = 1;
                telemetry.addLine("left");
                break;
            case CENTER:
                propPosition = 2;
                telemetry.addLine("center");
                break;
            case RIGHT:
                propPosition = 3;
                telemetry.addLine("right");
                break;
            case NONE:
                propPosition = 2;
                telemetry.addLine("none");
                break;
        }

        purplePixelDrop13(propPosition);

        while (opModeIsActive() && !isStopRequested() && !finishedAutonomous){

            switch (currentState){
                case PROP:
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.PREPARETODROPPIXEL;
                        prepareToPixelDrop23(propPosition, color);
                    }
                    telemetry.addData("prop", null);
                    break;
                case PREPARETODROPPIXEL:
                    telemetry.addData("PREPARETODROPPIXEL", null);
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.OPENSYSTEMS;
                        forbarDelay.startAction(GlobalData.currentTime);
                        encoderDelay.startAction(GlobalData.currentTime);
                    }
                    break;
                case OPENSYSTEMS:
                    elevatorStates = ElevatorStates.MIN;
                    telemetry.addData("forbarstate", fourbarState);
                    telemetry.addData("poselevator", Elevator.getPos());
                    if (forbarDelay.isDelayPassed()){
                        if (Elevator.reachedHeight(ElevatorConstants.autoHeight) || encoderDelay.isDelayPassed()){
                            dropYellowPixel23(propPosition, color);
                            currentState = AutonomousSteps.GOTOBOARD;
                        }
                        fourbarState = FourbarState.MOVE;
                    }

                    break;
                case GOTOBOARD:
                    if (!drive.isBusy()){
                        currentState = AutonomousSteps.DROPPIXEL;
                        dropYellowPixelDelay.startAction(GlobalData.currentTime);
                    }
                    break;
                case DROPPIXEL:
                    outtakeState = OuttakeState.TOWOUT;
                    if (dropYellowPixelDelay.isDelayPassed()){
                        currentState = AutonomousSteps.FARFROMTHEBOARD;
                        prepareToPixelDrop23(propPosition, color);
                    }
                    break;
                case FARFROMTHEBOARD:
                    if (!drive.isBusy()) {
                        currentState = AutonomousSteps.CLOSESYSTEMS;
                        elevatorClosingDelay.startAction(GlobalData.currentTime);
                        encoderDelay.isDelayPassed();
                    }
                    break;
                case CLOSESYSTEMS:
                    outtakeState = OuttakeState.CLOSED;
                    fourbarState = FourbarState.REVERSE;
                    if (elevatorClosingDelay.isDelayPassed()) {
                        elevatorStates = ElevatorStates.INTAKE;
                        if (Elevator.reachedHeight(Elevator.getPos()) || encoderDelay.isDelayPassed()){
                            currentState = AutonomousSteps.GOTOPARKING;
                            parking(robotPosition, parkingPos);
                        }
                    }
                    break;
                case GOTOPARKING:
                    if (!drive.isBusy()){
                        finishedAutonomous = true;
                    }
                  telemetry.addData("parked", null);
                  break;
            }
            GlobalData.currentTime = (float) time.seconds();
            Elevator.operateAutonomousNoMarker(elevatorStates, telemetry);
            Fourbar.operateTeleop(fourbarState);
            Outtake.operate(outtakeState);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            drive.update();
            telemetry.update();
        }


    }
}

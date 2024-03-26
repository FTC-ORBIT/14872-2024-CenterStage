//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.robotData.GlobalData;
//import org.firstinspires.ftc.teamcode.robotSubSystems.camera.RedPropThresholdClose;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
//import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
//import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
//import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//@Config
//@Autonomous (name = "RedClose")
//public class RedClose extends AutonomousGenaral{
//    private VisionPortal portal;
//    private final RedPropThresholdClose redPropThresholdClose = new RedPropThresholdClose();
//    ElapsedTime time = new ElapsedTime();
//    int propPositionTras = 1; //
//    int propPositionDirection = 1;
//    int robotPosition = 2;
//
//
//    boolean color = true;
//
//    String parkingPos = "close";
//    AutonomousSteps currentState;
//
//    ElevatorStates elevatorStates = ElevatorStates.INTAKE;
//
//    FourbarState fourbarState = FourbarState.REVERSE;
//
//    OuttakeState outtakeState = OuttakeState.CLOSED;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        redPropThresholdClose.initProp();
//
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .addProcessor(redPropThresholdClose)
//                .build();
//
//        init(hardwareMap);
//
//        waitForStart();
//        if (isStopRequested()) return;
//        time.reset();
//
//        currentState = AutonomousSteps.PROP;
//
//        telemetry.update();
//        switch (redPropThresholdClose.EnumGetPropPos()) {
//            case LEFT:
//                propPositionTras = 3;
//                propPositionDirection = 1;
//                telemetry.addLine("left");
//                break;
//            case CENTER:
//                propPositionTras = 2;
//                propPositionDirection = 2;
//                telemetry.addLine("center");
//                break;
//            case RIGHT:
//                propPositionTras = 1;
//                propPositionDirection = 3;
//                telemetry.addLine("right");
//                break;
//            case NONE:
//                propPositionTras = 2;
//                telemetry.addLine("none");
//                break;
//        }
//
//        purplePixelProp24(propPositionTras);
//
//        while (opModeIsActive() && !isStopRequested() && !finishedAutonomous){
//
//            switch (currentState){
//                case PROP:
//                    if (!drive.isBusy()){
//                        currentState = AutonomousSteps.PREPARETODROPPIXEL;
//                        prepareToPixelDrop23(propPositionDirection, color);
//                    }
//                    telemetry.addData("prop", null);
//                    break;
//                case PREPARETODROPPIXEL:
//                    telemetry.addData("PREPARETODROPPIXEL", null);
//                    if (!drive.isBusy()){
//                        currentState = AutonomousSteps.OPENSYSTEMS;
//                        forbarDelay.startAction(GlobalData.currentTime);
//                        encoderDelay.startAction(GlobalData.currentTime);
//                    }
//                    break;
//                case OPENSYSTEMS:
//                    elevatorStates = ElevatorStates.MIN;
//                    telemetry.addData("forbarstate", fourbarState);
//                    telemetry.addData("poselevator", Elevator.getPos());
//                    if (forbarDelay.isDelayPassed()){
//                        if (Elevator.reachedHeight(ElevatorConstants.autoHeight) || encoderDelay.isDelayPassed()){
//                            dropYellowPixel23(propPositionDirection, color);
//                            currentState = AutonomousSteps.GOTOBOARD;
//                        }
//                        fourbarState = FourbarState.MOVE;
//                    }
//
//                    break;
//                case GOTOBOARD:
//                    if (!drive.isBusy()){
//                        currentState = AutonomousSteps.DROPPIXEL;
//                        dropYellowPixelDelay.startAction(GlobalData.currentTime);
//                    }
//                    break;
//                case DROPPIXEL:
//                    outtakeState = OuttakeState.TOWOUT;
//                    if (dropYellowPixelDelay.isDelayPassed()){
//                        currentState = AutonomousSteps.FARFROMTHEBOARD;
//                        prepareToPixelDrop23(propPositionDirection, color);
//                    }
//                    break;
//                case FARFROMTHEBOARD:
//                    if (!drive.isBusy()) {
//                        currentState = AutonomousSteps.CLOSESYSTEMS;
//                        elevatorClosingDelay.startAction(GlobalData.currentTime);
//                        encoderDelay.startAction(GlobalData.currentTime);
//                    }
//                    break;
//                case CLOSESYSTEMS:
//                    outtakeState = OuttakeState.CLOSED;
//                    fourbarState = FourbarState.REVERSE;
//                    if (elevatorClosingDelay.isDelayPassed() ||encoderDelay.isDelayPassed()) {
//                        elevatorStates = ElevatorStates.INTAKE;
//                        if (Elevator.reachedHeight(Elevator.getPos())){
//                            currentState = AutonomousSteps.GOTOPARKING;
//                            parking(robotPosition, parkingPos);
//                        }
//                    }
//                    break;
//                case GOTOPARKING:
//                    if (!drive.isBusy()){
//                        finishedAutonomous = true;
//                    }
//                    telemetry.addData("parked", null);
//                    break;
//            }
//            GlobalData.currentTime = (float) time.seconds();
//            Elevator.operateAutonomousNoMarker(elevatorStates, telemetry);
//            Fourbar.operateTeleop(fourbarState);
//            Outtake.operate(outtakeState);
//            drive.update();
//            telemetry.update();
//        }
//
//
//    }
//    }
//

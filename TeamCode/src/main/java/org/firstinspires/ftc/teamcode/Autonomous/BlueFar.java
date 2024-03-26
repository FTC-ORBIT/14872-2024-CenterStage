//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.robotData.GlobalData;
//import org.firstinspires.ftc.teamcode.robotSubSystems.camera.BluePropThresholdFar;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
//import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
//import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
//import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//public class BlueFar extends AutonomousGenaral {
//
//    private VisionPortal portal;
//    private final BluePropThresholdFar bluePropThresholdFar = new BluePropThresholdFar();
//    ElapsedTime time = new ElapsedTime();
//    int position = 1;
//
//    boolean color = false;
//
//    String parkingPos = "far";
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
//        bluePropThresholdFar.initProp();
//
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .addProcessor(bluePropThresholdFar)
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
//        switch (bluePropThresholdFar.EnumGetPropPos()) {
//            case LEFT:
//                position = 3;
//                telemetry.addLine("left");
//                break;
//            case CENTER:
//                position = 2;
//                telemetry.addLine("center");
//                break;
//            case RIGHT:
//                position = 1;
//                telemetry.addLine("right");
//                break;
//            case NONE:
//                position = 2;
//                telemetry.addLine("none");
//                break;
//        }
//
//        purplePixelProp24(position);
//
//        while (opModeIsActive() && !isStopRequested()){
//
//            switch (currentState){
//                case PROP:
//                    if (!drive.isBusy()){
//                        currentState = AutonomousSteps.PREPARETODROPPIXEL;
////                        prepareToPixelDrop14(position, color);
//                    }
//                    telemetry.addData("prop", null);
//                    break;
//                case PREPARETODROPPIXEL:
//                    telemetry.addData("PREPARETODROPPIXEL", null);
//                    if (!drive.isBusy()){
//                        currentState = AutonomousSteps.OPENSYSTEMS;
//                        forbarDelay.startAction(GlobalData.currentTime);
//                    }
//                    break;
//                case OPENSYSTEMS:
//                    elevatorStates = ElevatorStates.MIN;
//                    telemetry.addData("forbarstate", fourbarState);
//                    telemetry.addData("poselevator", Elevator.getPos());
//                    if (forbarDelay.isDelayPassed()){
//                        if (Elevator.reachedHeight(ElevatorConstants.autoHeight)){
//                            dropYellowPixel23(position, color);
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
//                        time.reset();
//                        prepareToPixelDrop23(position, color);
//                    }
//                case FARFROMTHEBOARD:
//                    currentState = AutonomousSteps.CLOSESYSTEMS;
//                    break;
//                case CLOSESYSTEMS:
//                    outtakeState = OuttakeState.CLOSED;
//                    if (!drive.isBusy()){
//                        fourbarState = FourbarState.REVERSE;
//                        elevatorClosingDelay.startAction(GlobalData.currentTime);
//                    }
//                    if (elevatorClosingDelay.isDelayPassed()) {
//                        elevatorStates = ElevatorStates.INTAKE;
////                        if (Elevator.reachedHeight(Elevator.getPos())){
//                            currentState = AutonomousSteps.GOTOPARKING;
//                            parking(position, parkingPos);
//                        }
//                    }
//                    break;
//                case GOTOPARKING:
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
//}

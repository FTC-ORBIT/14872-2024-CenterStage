package org.firstinspires.ftc.teamcode.drive;//package org.firstinspires.ftc.teamcode.drive;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
//import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.center.CenterState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.center.ServoCenter;
//import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
//import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
//import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
//import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Config
//@Autonomous(name = "Test")
//public class AutonomousTest extends LinearOpMode {
//    public static double backwardSecond = 10;
//    public static double firstHighX = 52.5;
//    public static double firstHighY = -2;
//    public static double secondHighX = 47;
//    public static double secondHighY = 0;
//    public static double turnAngleHigh = -45;
//    public static double goToHighFirst = 9.3;
//    public static double goToHighSecond = 14.5;
//    public static double goToHighThird = 15;
//    public static double backWardsHighSecond = 10;
//    public static double backWardsHighThird = 0.5;
//    public static double coneStacksX = 50;
//    public static double coneStacksY = 26.5;
//    public static double thirdHighX = 45.5;
//    public static double thirdHighY = 0.5;
//    public static double coneStacks2X = 50;
//    public static double coneStacks2Y = 27;
//    public static double coneStacks3X = 49;
//    public static double coneStacks3Y = 27.5;
//    public static double firstDelayTime = 0.3;
//    public static double depleteDelay = 1;
//    public static double turnAngleHighThird = 0;
//    public static double fourthHighX = 44;
//    public static double fourthHighY = 0.5;
//    public static double goToHighFourth = 14.8;
//    public static double backWardsHighFourth = 12;
//    public static double fourthAdditionalAngle = 2;
//    public static double parking1X = 49;
//    public static double parking1Y = 27;
//    public static double parking3X = 42;
//    public static double parking3Y = -20;
//    public static double parkingNum = 3;
//
//
//    enum States {
//        FIRST,
//        SECOND,
//        THIRD,
//        FOURTH,
//        FIFTH,
//        SIXTH,
//        SEVENTH,
//        EIGHT,
//        NINTH,
//        TENTH,
//        ELEVENTH,
//        TWELVE,
//        THIRTEEN,
//        FOURTEEN,
//        FIFTEEN,
//        SIXTEEN,
//        SEVENTEEN,
//        EIGHTEEN,
//        IDLE
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Intake.init(hardwareMap);
//        Elevator.initAutonomous(hardwareMap);
//        Drivetrain.init(hardwareMap);
//        Intake.init(hardwareMap);
//        Claw.init(hardwareMap);
////        Arm.init(hardwareMap);
//        ServoCenter.init(hardwareMap);
//
//        Elevator.floor = 5;
//
//        ElapsedTime timer = new ElapsedTime();
//
//        TrajectorySequence firstDelay = drive.trajectorySequenceBuilder(new Pose2d())
//                .waitSeconds(AutonomousTest.firstDelayTime)
//                .build();
//
//
//        TrajectorySequence goToHighFirstTraj = drive.trajectorySequenceBuilder(firstDelay.end())
//                .lineToLinearHeading(new Pose2d(firstHighX, firstHighY, Math.toRadians(turnAngleHigh)))
//                .build();
//
//        TrajectorySequence goToDepleteFirst = drive.trajectorySequenceBuilder(goToHighFirstTraj.end())
//                .forward(goToHighFirst)
//                .build();
//
//        TrajectorySequence backFromHigh = drive.trajectorySequenceBuilder(goToDepleteFirst.end())
//                .back(backwardSecond)
//                .build();
//
//        TrajectorySequence toFirstConeStacks = drive.trajectorySequenceBuilder(backFromHigh.end())
//                .lineToSplineHeading(new Pose2d(coneStacksX, coneStacksY, Math.toRadians(-90)))
//                .build();
//
//        TrajectorySequence goToHighSecondTraj = drive.trajectorySequenceBuilder(toFirstConeStacks.end())
//                .lineToSplineHeading(new Pose2d(secondHighX, secondHighY, Math.toRadians(turnAngleHigh)))
//                .build();
//
//        TrajectorySequence goToDepleteHighSecond = drive.trajectorySequenceBuilder(goToHighSecondTraj.end())
//                .forward(goToHighSecond)
//                .build();
//
//        TrajectorySequence awayFromHighSecond = drive.trajectorySequenceBuilder(goToDepleteHighSecond.end())
//                .back(backWardsHighSecond)
//                .build();
//
//        TrajectorySequence goToConeStacksSecond = drive.trajectorySequenceBuilder(awayFromHighSecond.end())
//                .lineToSplineHeading(new Pose2d(coneStacks2X, coneStacks2Y, Math.toRadians(-90)))
//                .build();
//
//        TrajectorySequence goToHighThirdTraj = drive.trajectorySequenceBuilder(goToConeStacksSecond.end())
//                .lineToSplineHeading(new Pose2d(thirdHighX, thirdHighY, Math.toRadians(turnAngleHigh)))
//                .turn(Math.toRadians(turnAngleHighThird))
//                .build();
//
//        TrajectorySequence goToDepleteHighThird = drive.trajectorySequenceBuilder(goToHighThirdTraj.end())
//                .forward(goToHighThird)
//                .build();
//
//        TrajectorySequence awayFromHighThird = drive.trajectorySequenceBuilder(goToHighThirdTraj.end())
//                .back(backWardsHighThird)
//                .build();
//
//        TrajectorySequence goToConeStacksThirdTraj = drive.trajectorySequenceBuilder(awayFromHighThird.end())
//                .lineToSplineHeading(new Pose2d(coneStacks3X, coneStacks3Y, Math.toRadians(-90)))
//                .build();
//
//        TrajectorySequence goToHighFourthTraj = drive.trajectorySequenceBuilder(goToConeStacksThirdTraj.end())
//                .lineToSplineHeading(new Pose2d(fourthHighX, fourthHighY, Math.toRadians(turnAngleHigh + fourthAdditionalAngle)))
//                .build();
//
//        TrajectorySequence goToDepleteFourthTraj = drive.trajectorySequenceBuilder(goToHighFourthTraj.end())
//                        .forward(goToHighFourth)
//                                .build();
//
//        TrajectorySequence awayFromHighFourthTraj = drive.trajectorySequenceBuilder(goToDepleteFourthTraj.end())
//                        .back(backWardsHighFourth)
//                                .build();
//
//        TrajectorySequence goToParking1 = drive.trajectorySequenceBuilder(awayFromHighFourthTraj.end())
//                        .lineToSplineHeading(new Pose2d(parking1X, parking1Y, Math.toRadians(-90)))
//                                .build();
//
//
//        TrajectorySequence goToParking3 = drive.trajectorySequenceBuilder(awayFromHighFourthTraj.end())
//                .lineToSplineHeading(new Pose2d(parking3X, parking3Y, Math.toRadians(-90)))
//                .build();
//
//
//
//        waitForStart();
//
//        Claw.operate(ClawState.CLOSE);
//        ServoCenter.operate(CenterState.UP);
//        drive.followTrajectorySequence(firstDelay);
//        if (isStopRequested()) return;
//        States currentState = States.FIRST;
//        drive.followTrajectorySequenceAsync(goToHighFirstTraj);
//
//        while (opModeIsActive() && !isStopRequested()){
//            switch (currentState) {
//                case FIRST:
//                    Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
//                    Claw.operate(ClawState.CLOSE);
//                    Arm.operate(ArmState.BACK);
//                    timer.reset();
//                    if (!drive.isBusy()){
//                        currentState = States.SECOND;
//                        drive.followTrajectorySequenceAsync(goToDepleteFirst);
//                    }
//                    telemetry.addData("first", null);
//                    break;
//                case SECOND:
//                    ServoCenter.operate(CenterState.DOWN);
//                    Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
//                    Intake.operate(IntakeState.COLLECT);
//                    if (!drive.isBusy()){
//                        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
//                        if (timer.seconds() >= depleteDelay) {
//                            Claw.operate(ClawState.OPEN);
//                            currentState = States.THIRD;
//                            drive.followTrajectorySequenceAsync(backFromHigh);
//                        }
//                    }
//                    telemetry.addData("second", null);
//                    break;
//                case THIRD:
//                    Elevator.operateAutonomous(ElevatorStates.MID, telemetry);
//                    Intake.operate(IntakeState.STOP);
//                    if (!drive.isBusy()){
//                        Arm.operate(ArmState.FRONT);
//                        drive.followTrajectorySequenceAsync(toFirstConeStacks);
//                        currentState = States.FOURTH;
//                    }
//                    telemetry.addData("third", null);
//                    break;
//                case FOURTH:
//                    Arm.operate(ArmState.FRONT);
//                    Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry);
//                    if (!drive.isBusy()){
//                        Claw.operate(ClawState.CLOSE);
//                        currentState = States.FIFTH;
//                    }
//                    telemetry.addData("fourth", null);
//                    timer.reset();
//                    break;
//                case FIFTH:
//                    Elevator.setWanted(ElevatorConstants.lowHeight);
//                    if (timer.seconds() >= firstDelayTime){
//                        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry);
//                    }
//                    if (Elevator.reachedHeight()){
//                        Arm.operate(ArmState.BACK);
//                        drive.followTrajectorySequenceAsync(goToHighSecondTraj);
//                        currentState = States.SIXTH;
//                }
//                    break;
//                case SIXTH:
//                    Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
//                    if (!drive.isBusy()){
//                        drive.followTrajectorySequenceAsync(goToDepleteHighSecond);
//                        currentState = States.SEVENTH;
//                    }
//                    timer.reset();
//                    break;
//                case SEVENTH:
//                    if (drive.isBusy()) {
//                        Intake.operate(IntakeState.COLLECT);
//                        timer.reset();
//                    }
//                    if (!drive.isBusy()){
//                        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
//                        Claw.operate(ClawState.OPEN);
//                        Intake.operate(IntakeState.STOP);
//                        drive.followTrajectorySequenceAsync(awayFromHighSecond);
//                        currentState = States.EIGHT;
//                    }
//                    break;
//                case EIGHT:
//                    Elevator.coneStacksFloor = 4;
//                    Intake.operate(IntakeState.STOP);
//                    Elevator.operateAutonomous(ElevatorStates.MID, telemetry);
//                    if (!drive.isBusy()){
//                        Arm.operate(ArmState.FRONT);
//                        drive.followTrajectorySequenceAsync(goToConeStacksSecond);
//                        currentState = States.NINTH;
//                    }
//                    break;
//                case NINTH:
//                    if (drive.isBusy()) {
//                        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry);
//                        timer.reset();
//                        telemetry.addData("busy", null);
//                    }
//                    if (!drive.isBusy()){
//                        Claw.operate(ClawState.CLOSE);
//                        if (timer.seconds() > firstDelayTime){
//                            Elevator.operateAutonomous(ElevatorStates.LOW, telemetry);
//                            if (Elevator.reachedHeight()){
//                                currentState = States.TENTH;
//                                drive.followTrajectorySequenceAsync(goToHighThirdTraj);
//                            }
//                        }
//                        telemetry.addData("isn't busy", null);
//                    }
//                    break;
//                case TENTH:
//                    Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
//                    Arm.operate(ArmState.BACK);
//                    if (!drive.isBusy()){
//                        drive.followTrajectorySequenceAsync(goToDepleteHighThird);
//                        currentState = States.ELEVENTH;
//                        Claw.operate(ClawState.CLOSE);
//                    }
//                    break;
//                case ELEVENTH:
//                    if (drive.isBusy()) {
//                        Intake.operate(IntakeState.COLLECT);
//                    }
//                    if (!drive.isBusy()){
//                        Intake.operate(IntakeState.DEPLETE);
//                        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
//                        Claw.operate(ClawState.OPEN);
//                        drive.followTrajectorySequenceAsync(awayFromHighThird);
//                        currentState = States.TWELVE;
//                    }
//                    break;
//                case TWELVE:
//                    Elevator.operateAutonomous(ElevatorStates.MID, telemetry);
//                    Intake.operate(IntakeState.DEPLETE);
//                    if (!drive.isBusy()){
//                        currentState = States.THIRTEEN;
//                        drive.followTrajectorySequenceAsync(goToConeStacksThirdTraj);
//                    }
//                    break;
//                case THIRTEEN:
//                    if (drive.isBusy()) {
//                        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry);
//                        Arm.operate(ArmState.FRONT);
//                        Intake.operate(IntakeState.STOP);
//                        timer.reset();
//                    }
//                    if (!drive.isBusy()){
//                        Claw.operate(ClawState.CLOSE);
//                        if (timer.seconds() >= firstDelayTime) {
//                            Elevator.operateAutonomous(ElevatorStates.LOW, telemetry);
//                            if (Elevator.reachedHeight()){
//                                drive.followTrajectorySequenceAsync(goToHighFourthTraj);
//                                currentState = States.FIFTEEN;
//                            }
//                        }
//                        telemetry.addData("fourteen", null);
//                        telemetry.addData("elevator", Elevator.reachedHeight());
//                    }
//                    break;
//                case FIFTEEN:
//                    telemetry.addData("fifteen", null);
//                    Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
//                    Arm.operate(ArmState.BACK);
//                    if (!drive.isBusy()){
//                        drive.followTrajectorySequenceAsync(goToDepleteFourthTraj);
//                        currentState = States.SIXTEEN;
//                    }
//                    break;
//                case SIXTEEN:
//                    if (drive.isBusy()) {
//                        Intake.operate(IntakeState.COLLECT);
//                    }
//                    if (!drive.isBusy()) {
//                        Intake.operate(IntakeState.DEPLETE);
//                        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
//                        Claw.operate(ClawState.OPEN);
//                        drive.followTrajectorySequenceAsync(awayFromHighFourthTraj);
//                        currentState = States.SEVENTEEN;
//                    }
//                    break;
//                case SEVENTEEN:
//                    Elevator.operateAutonomous(ElevatorStates.MID, telemetry);
//                    Intake.operate(IntakeState.DEPLETE);
//                    if (!drive.isBusy()) {
//                        if (parkingNum == 1){
//                            drive.followTrajectorySequenceAsync(goToParking1);
//                            currentState = States.EIGHTEEN;
//                        } else if (parkingNum == 2){
//                            currentState = States.IDLE;
//                        } else {
//                            drive.followTrajectorySequenceAsync(goToParking3);
//                            currentState = States.EIGHTEEN;
//                        }
//                    }
//                    break;
//                case EIGHTEEN:
//                    Intake.operate(IntakeState.STOP);
//                case IDLE:
//                    Arm.operate(ArmState.FRONT);
//                    Elevator.operateAutonomous(ElevatorStates.GROUND, telemetry);
//                    Intake.operate(IntakeState.STOP);
//                    break;
//            }
//            drive.update();
//            telemetry.update();
//        }
//    }
//}
//

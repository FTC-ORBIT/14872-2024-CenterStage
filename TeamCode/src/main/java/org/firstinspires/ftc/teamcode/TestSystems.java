package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
//import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
//import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
//import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
//import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.center.ServoCenter;
//import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
//import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
//import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
//import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
//
//
//@TeleOp(name = "TestSystems")
//public class TestSystems extends LinearOpMode {
//    ElevatorStates elevatorState = ElevatorStates.GROUND;
//      boolean lastRightBumper;
//    public static DigitalChannel coneDistanceSensor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Claw.init(hardwareMap);
//        Intake.init(hardwareMap);
////        Arm.init(hardwareMap);
//        Drivetrain.init(hardwareMap);
//        Elevator.initAutonomous(hardwareMap);
//        OrbitGyro.init(hardwareMap);
//        ServoCenter.init(hardwareMap);
//        coneDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
//        coneDistanceSensor.setMode(DigitalChannel.Mode.INPUT);
//
//
//        waitForStart();
//        while (!isStopRequested()) {
//            if (gamepad1.a) Claw.operate(ClawState.OPEN);
//            else if (gamepad1.b) Claw.operate(ClawState.CLOSE);
//            if (gamepad1.x) Arm.operate(ArmState.BACK);
//            else if (gamepad1.y) Arm.operate(ArmState.FRONT);
//            if (gamepad1.dpad_left) Intake.operate(IntakeState.COLLECT);
//            else if (gamepad1.dpad_down) Intake.operate(IntakeState.STOP);
//            else if (gamepad1.dpad_right) Intake.operate(IntakeState.DEPLETE);
//
//            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2)
//                Drivetrain.operate(new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
//
//            ServoCenter.testArm(gamepad1, telemetry);
//
//            telemetry.update();
//            telemetry.addData("distanceSensor", coneDistanceSensor.getState());
//            telemetry.addData("parallelEncoder", Drivetrain.motors[2].getCurrentPosition());
//            telemetry.addData("perpandicularEncoder", Drivetrain.motors[3].getCurrentPosition());
//            telemetry.addData("elevatorEncoder", Drivetrain.motors[1].getCurrentPosition());
//
//
//            lastRightBumper = gamepad1.right_bumper;
//        }
//    }
//}

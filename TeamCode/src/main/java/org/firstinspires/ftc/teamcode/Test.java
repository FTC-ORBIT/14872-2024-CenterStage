package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.opencv.android.FpsMeter;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OrbitGyro.init(hardwareMap);
        Elevator.init(hardwareMap);
        Outtake.init(hardwareMap);
        Intake.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Plane.init(hardwareMap);

        while (!isStopRequested()) {
            Vector leftStick = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            float omega = gamepad1.right_trigger - gamepad1.left_trigger;
            Drivetrain.operate(leftStick,  omega , telemetry , gamepad1);
          Elevator.test(gamepad1, telemetry);
            Outtake.test(gamepad1, telemetry);
            Fourbar.test(gamepad1 , telemetry);
            telemetry.addData("Fourbar pos", Fourbar.servo.getPosition());
          telemetry.addData("pos", Elevator.elevatorMotor.getCurrentPosition());
          telemetry.addData("pos2" , Elevator.elevatorMotor2.getCurrentPosition());
            telemetry.update();
        }


// intake: 0
// low: 2679 , 2749
// mid:
// high: ?
    }
}
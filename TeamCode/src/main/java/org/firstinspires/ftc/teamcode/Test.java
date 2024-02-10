package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
//    FourbarState state1 = FourbarState.REVERSE;

    @Override
    public void runOpMode() throws InterruptedException {
//        Fourbar.init(hardwareMap);
        Intake.init(hardwareMap);
//            waitForStart();
        while (!isStopRequested()) {
            Intake.test(gamepad1, telemetry);
//          Fourbar.test(gamepad1 , telemetry);
            // if you want to have the old fourbar test comment all of these lines:
//          Fourbar.operate(state1, gamepad1,telemetry);
//          if (gamepad1.a){
//              state1 = FourbarState.REVERSE;
//          } else if (gamepad1.x){
//              state1 = FourbarState.MOVE;
//          } else if (gamepad1.y){
//              state1 = FourbarState.MOVETOMID;
//          }
            //until here

//          telemetry.addData("pos" , Fourbar.servo.getPosition());
            telemetry.addData("pos", Intake.motor.getCurrentPosition());
            telemetry.update();
        }


// intake: 0
// low: 2679 , 2749
// mid:
// high: ?
    }
}
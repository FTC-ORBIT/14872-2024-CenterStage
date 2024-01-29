package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
      Intake.init(hardwareMap);
      Outtake.init(hardwareMap);

      waitForStart();
      while (!isStopRequested()){
          Outtake.test(gamepad1, telemetry);
          Intake.test(gamepad1 , telemetry);
          telemetry.addData("pos", Intake.motor.getCurrentPosition());
          telemetry.addData("outtake" , Outtake.servo.getPosition());
          telemetry.update();
      }
  }
}
// intake: -3
// low: 1894
// mid: 3270
// high: ?
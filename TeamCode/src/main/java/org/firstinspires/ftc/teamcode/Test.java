package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
      Intake.init(hardwareMap);

      waitForStart();
      while (!isStopRequested()){
          Intake.test(gamepad1 , telemetry);
          telemetry.addData("power", Intake.motor.getPower());
          telemetry.update();
      }
  }
}
// intake: -3
// low: 1894
// mid: 3270
// high: ?
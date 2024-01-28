package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
      Fourbar.init(hardwareMap);

      waitForStart();
      while (!isStopRequested()){
          Fourbar.test(gamepad1 , telemetry);
          telemetry.addData("pos", Fourbar.servo.getPosition());
          telemetry.update();
      }
  }
}
// intake: 0 , 0
// low: 3000 , -3000
// mid: 3000 , -3000
// high: ?
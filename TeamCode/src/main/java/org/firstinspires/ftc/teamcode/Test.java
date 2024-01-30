package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
      Elevator.init(hardwareMap);
            waitForStart();
      while (!isStopRequested()){
          Elevator.test(gamepad1 , telemetry);
          telemetry.addData("elevator pos" , Elevator.elevatorMotor.getCurrentPosition());
          telemetry.addData("elevator2 pos" , Elevator.elevatorMotor2.getCurrentPosition());
          telemetry.update();
      }
  }
}
// intake: 0
// low: 2635 , 2468
// mid: 3047 , 2910
// high: ?
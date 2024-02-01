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
          telemetry.addData("pos" , Elevator.elevatorMotor.getCurrentPosition());
          telemetry.addData("pos2" , Elevator.elevatorMotor2.getCurrentPosition());
          telemetry.addData("pid pos" , Elevator.elevatorPID.update(Elevator.currentPos, telemetry));
          telemetry.addData("pid pos2" , Elevator.elevatorPID.update(Elevator.currentPos2, telemetry));
          telemetry.update();
      }
  }
}
// intake: 0
// low: 2679 , 2749
// mid:
// high: ?
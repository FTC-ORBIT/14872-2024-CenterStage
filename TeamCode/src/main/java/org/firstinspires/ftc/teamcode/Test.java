package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
    ElevatorStates state = ElevatorStates.INTAKE;
  @Override
  public void runOpMode() throws InterruptedException {
      Elevator.init(hardwareMap);
            waitForStart();
      while (!isStopRequested()){
        Elevator.operate(state, gamepad1, telemetry);
//          Elevator.test(gamepad1, telemetry);
          if (gamepad1.a){
              state = ElevatorStates.INTAKE;
          } else if (gamepad1.b){
              state = ElevatorStates.LOW;
          } else if (gamepad1.y){
              state= ElevatorStates.MID;
          }
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
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.OrbitLED;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {
    ElevatorStates state = ElevatorStates.INTAKE;
  @Override
  public void runOpMode() throws InterruptedException {
      OrbitLED.init(hardwareMap);
            waitForStart();
      while (!isStopRequested()){
          OrbitLED.operate();
          telemetry.addData("pos" , Elevator.elevatorMotor.getCurrentPosition());
          telemetry.addData("pos2", Elevator.elevatorMotor2.getCurrentPosition());
          telemetry.update();
      }
  }
}
// intake: 0
// low: 2679 , 2749
// mid:
// high: ?
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
      Drivetrain.init(hardwareMap);
      Intake.init(hardwareMap);
            waitForStart();
        while (!isStopRequested()) {
          Drivetrain.testEncoder(telemetry);
          Intake.test(telemetry);
            telemetry.update();
        }


// intake: 0
// low: 2679 , 2749
// mid:
// high: ?
    }
}
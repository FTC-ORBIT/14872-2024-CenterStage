package org.firstinspires.ftc.teamcode.roadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

@Autonomous(name = "test_1_M")
public
class test_1_M extends LinearOpMode {

    Drivetrain drivetrain = new Drivetrain();

    @Override

    public void runOpMode() {
        drivetrain.init(hardwareMap);

        telemetry.addLine("init finished");
        telemetry.update();

        waitForStart();

        sleep(500);

        drive_1_M();

        telemetry.addData("robot-pos" , PoseStorage::new);
        telemetry.update();
    }

    public void drive_1_M() {
        drivetrain.driveToDirection(100,0,0.5);
    }









}



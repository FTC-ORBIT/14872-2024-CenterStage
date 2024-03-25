package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.opencv.android.FpsMeter;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
//        coneDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
//        coneDistanceSensor.setMode(DigitalChannel.Mode.INPUT);

        ElapsedTime robotTime = new ElapsedTime();
        robotTime.reset();
//           Fixpixel.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        Elevator.init(hardwareMap);
        Outtake.init(hardwareMap);
        Intake.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Plane.init(hardwareMap);
//         OrbitLED.init(hardwareMap);
        OrbitColorSensor.init(hardwareMap);

        OrbitGyro.resetGyroStartTeleop((float) Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.update();
        telemetry.addData("gyro", Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.update();

        GlobalData.inAutonomous = false;
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.hasGamePiece = false;



        waitForStart();

        while (!isStopRequested()) {
            Outtake.test(gamepad1, telemetry);
            }

        }


// intake: 0
// low: 2679 , 2749
// mid:
// high: ?
    }
//dani yalechan!
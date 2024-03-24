package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// Red far = 1
// Red close = 2
// Blue close = 3
// Blue far = 4
@Config
public abstract class AutonomousGenaral extends LinearOpMode {

    public static ElapsedTime time = new ElapsedTime();

    public static SampleMecanumDrive drive;
    public static Pose2d startPos = new Pose2d(0,0,0);
    public static double maxVeloDrop = 5;
    public static TrajectoryVelocityConstraint velConstraintDrop = SampleMecanumDrive.getVelocityConstraint(maxVeloDrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint accConstraintDrop = SampleMecanumDrive.getAccelerationConstraint(maxVeloDrop);

    public static Pose2d lastTrajectoryPos = new Pose2d();

    // this is were i defined my variables
    public static double farFromTrasXDrop = 22.5;

    public static double farFromTrasYDropAndAfterProp = 9;

    public static double farFromTrasXAfterProp = 3;
    public static double centerXDrop = 29.7;
    public static double centerXAfterProp = 25.7;
    public static double splineXProp = 31;
    public static double splineYProp = 0.5;
    public static double splineEndAngle = Math.toRadians(-90);
    public static double splineEndTangent = Math.toRadians(120);
    public static double splineAfterPropY = 4;
    public static double turnAfterProp = Math.toRadians(90);

    public static double boardX1Red = 37;
    public static double boardX2Red = 34.7;
    public static double boardX3Red = 32;
    public static double boardX4Red = 29.2;
    public static double boardX5Red = 27;
    public static double boardX6Red = 22.5;

    public static double boardX6Blue = 37;
    public static double boardX5Blue = 34.7;
    public static double boardX4Blue = 32;
    public static double boardX3Blue = 29.2;
    public static double boardX2Blue = 27;
    public static double boardX1Blue = 22.5;
    public static double prepareToDropPixelY = 24;
    public static double boardPosY = 34;
    public static double dropYellowPixelDelay = 1;
    public static double elevatorClosingDelay = 1; // so there wonwt be any problem with the fourbar
    public static boolean parkingPos = true; // true = close to the wall
    public static double parkingY23 = 39; // when 2 = -39
    public static double parkingY14 = 83; // when 1 = -83
    public static double parkingX1234FarFromTheWall = 46;
    public static double parkingX1234CloseToTheWall = 3;
    //this is where I defined my poses
    public static Pose2d farFromTrasDropPose13 = new Pose2d(farFromTrasXDrop, farFromTrasYDropAndAfterProp, startPos.getHeading());
    public static Pose2d farFromTrasPosAfterProp13 = new Pose2d(farFromTrasXAfterProp, farFromTrasYDropAndAfterProp, startPos.getHeading());
    public static Pose2d centerPropPose = new Pose2d(centerXDrop, startPos.getY(), startPos.getHeading());
    public static Pose2d centerPosAfterProp = new Pose2d(centerXAfterProp, startPos.getY(),startPos.getHeading());
    public static Pose2d splinePropPos13 = new Pose2d(splineXProp, splineYProp, splineEndAngle);
    public static Pose2d closeTrasPosAfterProp13 = new Pose2d(splineXProp, -splineAfterPropY, splineEndAngle);
    public static Pose2d farFromTrasDropPose24 = new Pose2d(farFromTrasXDrop, -farFromTrasYDropAndAfterProp, startPos.getHeading());
    public static Pose2d farFromTrasPosAfterProp24 = new Pose2d(farFromTrasXAfterProp, -farFromTrasYDropAndAfterProp, startPos.getHeading());
    public static Pose2d splinePropPos24 = new Pose2d(splineXProp, splineYProp, splineEndAngle);
    public static Pose2d closeTrasPosAfterProp24 = new Pose2d(splineXProp, splineAfterPropY, splineEndAngle);

    public static Pose2d prepareToDropPixelPos1Red = new Pose2d(boardX1Red, -prepareToDropPixelY, turnAfterProp);
    public static Pose2d prepareToDropPixelPos4Red = new Pose2d(boardX4Red, -prepareToDropPixelY, turnAfterProp);
    public static Pose2d prepareToDropPixelPos6Red = new Pose2d(boardX6Red, -prepareToDropPixelY, turnAfterProp);

    public static Pose2d prepareToDropPixelPos1Blue = new Pose2d(boardX1Blue, prepareToDropPixelY, turnAfterProp);
    public static Pose2d prepareToDropPixelPos4Blue = new Pose2d(boardX4Blue, prepareToDropPixelY, turnAfterProp);
    public static Pose2d prepareToDropPixelPos6Blue = new Pose2d(boardX6Blue, prepareToDropPixelY, turnAfterProp);
    public static Pose2d dropYellowPixel1PosRed = new Pose2d(boardX1Red, -boardPosY , turnAfterProp);
    public static Pose2d dropYellowPixel4PosRed = new Pose2d(boardX4Red, -boardPosY, turnAfterProp);
    public static Pose2d dropYellowPixel6PosRed = new Pose2d(boardX6Red, -boardPosY, turnAfterProp);
    public static Pose2d dropYellowPixel1PosBlue = new Pose2d(boardX1Blue, boardPosY, turnAfterProp);
    public static Pose2d dropYellowPixel4PosBlue = new Pose2d(boardX4Blue,boardPosY, turnAfterProp);
    public static Pose2d dropYellowPixel6PosBlue = new Pose2d(boardX6Blue,boardPosY,turnAfterProp);
    public static Pose2d parking1CloseToTheWall = new Pose2d(parkingX1234CloseToTheWall, -parkingY14, startPos.getHeading());
    public static Pose2d parking1FarFromTheWall = new Pose2d(parkingX1234FarFromTheWall, -parkingY14, startPos.getHeading());
    public static Pose2d parking2CloseToTheWall = new Pose2d(parkingX1234CloseToTheWall, -parkingY23, startPos.getHeading());
    public static Pose2d parking2FarFromTheWall = new Pose2d(parkingX1234FarFromTheWall, -parkingY23, startPos.getHeading());
    public static Pose2d parking3CloseToTheWall = new Pose2d(parkingX1234CloseToTheWall, parkingY23, startPos.getHeading());
    public static Pose2d parking3FarFromTheWall = new Pose2d(parkingX1234FarFromTheWall, parkingY23, startPos.getHeading());
    public static Pose2d parking4CloseToTheWall = new Pose2d(parkingX1234CloseToTheWall, parkingY14, startPos.getHeading());
    public static Pose2d parking4FarFromTheWall = new Pose2d(parkingX1234FarFromTheWall, parkingY14, startPos.getHeading());



    public static void init (HardwareMap hardwareMap){
        time.reset();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);
        Elevator.init(hardwareMap);
        Fourbar.init(hardwareMap);
        Intake.init(hardwareMap);
        Outtake.init(hardwareMap);
    }

    //the position of the prop is defined like this:
    // 1 = far from tras
    // 2 = center
    // 3 = close to tras
    public static void purplePixelDrop13 (int position){
        if (position == 1){
            TrajectorySequence purplePixelDropFarFromTras = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(farFromTrasDropPose13)
                    .lineToLinearHeading(farFromTrasPosAfterProp13)
                    .turn(turnAfterProp)
                    .build();
            lastTrajectoryPos = purplePixelDropFarFromTras.end();
            drive.followTrajectorySequenceAsync(purplePixelDropFarFromTras);
        }
        else if (position == 2) {
            TrajectorySequence purplePixelDropCenter = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(centerPropPose)
                    .lineToLinearHeading(centerPosAfterProp)
                    .turn(turnAfterProp)
                    .build();
            lastTrajectoryPos = purplePixelDropCenter.end();
            drive.followTrajectorySequenceAsync(purplePixelDropCenter);
        } else if (position == 3) {
            TrajectorySequence purplePixelDropCloseToTras = drive.trajectorySequenceBuilder(startPos)
                    .splineToLinearHeading(splinePropPos13, splineEndTangent)
                    .lineToLinearHeading(closeTrasPosAfterProp13)
                    .turn(turnAfterProp)
                    .build();
            lastTrajectoryPos = purplePixelDropCloseToTras.end();
            drive.followTrajectorySequenceAsync(purplePixelDropCloseToTras);
        }

    }

    //the position of the prop is defined like this:
    // 1 = far from tras
    // 2 = center
    // 3 = close to tras
    public static void  purplePixelProp24(int position){
        if (position == 1){
            TrajectorySequence purplePixelDropFarFromTras = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(farFromTrasDropPose24)
                    .lineToLinearHeading(farFromTrasPosAfterProp24)
                    .turn(-turnAfterProp)
                    .build();
            lastTrajectoryPos = purplePixelDropFarFromTras.end();
            drive.followTrajectorySequenceAsync(purplePixelDropFarFromTras);
        } else if (position == 2) {
            TrajectorySequence purplePixelDropCenter = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(centerPropPose)
                    .lineToLinearHeading(centerPosAfterProp)
                    .turn(-turnAfterProp)
                    .build();
            lastTrajectoryPos = purplePixelDropCenter.end();
            drive.followTrajectorySequenceAsync(purplePixelDropCenter);
        } else if (position == 3) {
            TrajectorySequence purplePixelDropCloseToTras = drive.trajectorySequenceBuilder(startPos)
                    .splineToLinearHeading(splinePropPos24,splineEndTangent)
                    .lineToLinearHeading(closeTrasPosAfterProp24)
                    .turn(-turnAfterProp)
                    .build();
            lastTrajectoryPos = purplePixelDropCloseToTras.end();
            drive.followTrajectorySequenceAsync(purplePixelDropCloseToTras);
        }
    }

    //true = blue
    // false = red
    //the position of the prop is defined like this:
    // 1 = left
    // 2 = center
    // 3 = right
    public static void prepareToPixelDrop14(){

    }

    //true = red
    // false = blue
    //the position of the prop is defined like this:
    // 1 = left
    // 2 = center
    // 3 = right

    //also used after yellow pixel drop
    public static void prepareToPixelDrop23(int position, boolean color){
     if (position == 1 && color){
         TrajectorySequence prepareToPixelDropLeft = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                 .lineToLinearHeading(prepareToDropPixelPos1Red)
                 .build();
         lastTrajectoryPos = prepareToPixelDropLeft.end();
         drive.followTrajectorySequenceAsync(prepareToPixelDropLeft);
     } else if (position == 1){
         TrajectorySequence prepareTopixelDropLeft = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                 .lineToLinearHeading(prepareToDropPixelPos1Blue)
                 .build();
         lastTrajectoryPos = prepareTopixelDropLeft.end();
         drive.followTrajectorySequenceAsync(prepareTopixelDropLeft);
     } else if (position == 2 && color){
         TrajectorySequence prepareToPixelDropCenter = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                 .lineToLinearHeading(prepareToDropPixelPos4Red)
                 .build();
         lastTrajectoryPos = prepareToPixelDropCenter.end();
         drive.followTrajectorySequenceAsync(prepareToPixelDropCenter);
     } else if (position == 2){
         TrajectorySequence prepareToPixelDropCenter = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                 .lineToLinearHeading(prepareToDropPixelPos4Blue)
                 .build();
         lastTrajectoryPos = prepareToPixelDropCenter.end();
         drive.followTrajectorySequenceAsync(prepareToPixelDropCenter);
     } else if (position == 3 && color) {
         TrajectorySequence prepareToPixelDropRight = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                 .lineToLinearHeading(prepareToDropPixelPos6Red)
                 .build();
         lastTrajectoryPos = prepareToPixelDropRight.end();
         drive.followTrajectorySequenceAsync(prepareToPixelDropRight);
     }else if (position == 3){
         TrajectorySequence prepareToPixelDropRight = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                 .lineToLinearHeading(prepareToDropPixelPos6Blue)
                 .build();
         lastTrajectoryPos = prepareToPixelDropRight.end();
         drive.followTrajectorySequenceAsync(prepareToPixelDropRight);
     }
    }
    public static void markerBoard(Telemetry telemetry ){
        TrajectorySequence Marker = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                .addTemporalMarker(() -> {
                    Elevator.operateAutonomous(ElevatorStates.AUTO,telemetry);
                })
                .addTemporalMarker(() -> {
                    Fourbar.operateAutonomous(FourbarState.MOVE);
                })
                .build();
        lastTrajectoryPos = Marker.end();
        drive.followTrajectorySequenceAsync(Marker);
    }
    //true = red
    // false = blue
    //the position of the prop is defined like this:
    // 1 = left
    // 2 = center
    // 3 = right
    public static void dropYellowPixel23(int position, boolean color){
        if (position == 1 && color){
            TrajectorySequence dropYellowPixelLeft = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .setConstraints(velConstraintDrop, accConstraintDrop)
                    .lineToLinearHeading(dropYellowPixel1PosRed)
                    .resetConstraints()
                    .build();
            lastTrajectoryPos = dropYellowPixelLeft.end();
            drive.followTrajectorySequenceAsync(dropYellowPixelLeft);
        } else if (position == 1) {
            TrajectorySequence dropYellowPixelLeft = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .setConstraints(velConstraintDrop, accConstraintDrop)
                    .lineToLinearHeading(dropYellowPixel1PosBlue)
                    .resetConstraints()
                    .build();
            lastTrajectoryPos = dropYellowPixelLeft.end();
            drive.followTrajectorySequenceAsync(dropYellowPixelLeft);
        } else if (position == 2 && color) {
            TrajectorySequence dropYellowPixelCenter = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .setConstraints(velConstraintDrop, accConstraintDrop)
                    .lineToLinearHeading(dropYellowPixel4PosRed)
                    .resetConstraints()
                    .build();
            lastTrajectoryPos = dropYellowPixelCenter.end();
            drive.followTrajectorySequenceAsync(dropYellowPixelCenter);
        } else if (position == 2) {
            TrajectorySequence dropYellowPixelCenter = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .setConstraints(velConstraintDrop, accConstraintDrop)
                    .lineToLinearHeading(dropYellowPixel4PosBlue)
                    .resetConstraints()
                    .build();
            lastTrajectoryPos = dropYellowPixelCenter.end();
            drive.followTrajectorySequenceAsync(dropYellowPixelCenter);
        } else if (position == 3 && color) {
            TrajectorySequence dropYellowPixelRight = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .setConstraints(velConstraintDrop, accConstraintDrop)
                    .lineToLinearHeading(dropYellowPixel6PosRed)
                    .resetConstraints()
                    .build();
            lastTrajectoryPos = dropYellowPixelRight.end();
            drive.followTrajectorySequenceAsync(dropYellowPixelRight);
        } else if (position == 3) {
            TrajectorySequence dropYellowPixelRight = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .setConstraints(velConstraintDrop, accConstraintDrop)
                    .lineToLinearHeading(dropYellowPixel6PosBlue)
                    .resetConstraints()
                    .build();
            lastTrajectoryPos = dropYellowPixelRight.end();
            drive.followTrajectorySequenceAsync(dropYellowPixelRight);
        }
    }
    public static void parking(int position, String farOrClose){
        if (position == 1 && farOrClose.equals("far")){
            TrajectorySequence parking1Far = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking1FarFromTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking1Far);
        } else if (position == 1 ) {
            TrajectorySequence parking1Close = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking1CloseToTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking1Close);
        } else if (position == 2 && farOrClose.equals("far")) {
            TrajectorySequence parking2Far = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking2FarFromTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking2Far);
        } else if (position == 2) {
            TrajectorySequence parking2Close = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking2CloseToTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking2Close);
        } else if (position == 3 && farOrClose.equals("far")) {
            TrajectorySequence parking3Far = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking3FarFromTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking3Far);
        } else if (position == 3) {
            TrajectorySequence parking3Close = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking3CloseToTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking3Close);
        } else if (position == 4 && farOrClose.equals("far")) {
            TrajectorySequence parking4Far = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking4FarFromTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking4Far);
        }else if (position == 4){
            TrajectorySequence parking4Close = drive.trajectorySequenceBuilder(lastTrajectoryPos)
                    .lineToLinearHeading(parking4CloseToTheWall)
                    .build();
            drive.followTrajectorySequenceAsync(parking4Close);
        }
    }
}
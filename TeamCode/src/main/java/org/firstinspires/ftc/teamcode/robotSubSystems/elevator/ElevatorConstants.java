package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElevatorConstants {
    public static final float groundHeight = -100;
    public static final float intakeHeight = groundHeight;
    public static final float lowHeight = 15200;
    public static final float midHeight = 23885;
    public static final float highHeight = 32900;//33000
    public static final float coneStacksHeight = 4600; //1586
    public static final float coneStacksDifference = 1000;
    public static final float ableToOpenClawHeight = 6500;
    public static final float ableToTurnArmHeight = 10000;
    public static final float constantPower = 0.074f;
    public static final float kP = (float) 0.001;
    public static final float kD = (float) 0;
    public static final float depletePower = -0.7f;
    public static final float wantedDeplete = Elevator.height - 4000;
    //public static final Vector minMetersToTicks = new Vector(0 , 0);
    //public static final Vector maxMetersToTicks = new Vector(0.86f, 33025);
    //dani yalechan
//6339
    //2000

}


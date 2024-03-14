package org.firstinspires.ftc.teamcode.robotData;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
//import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;

public  class Constants {
    public static  final float teleopCycleTime = 0.02f; //TODO tune
    public static final float INF = 1e5f;
    public static final float epsilon = 1e-5f;
    public static final float[] redCone = {115, 120, 75};
    public static final float[] blueCone = {50, 103, 115};
    public static final float colorRange = 25;
    public static  final float minRotationTranslationRatio = 0.2f;
    public static  final float robotRadius = 0.2f;
    public static  final float maxAcc = 1.2f;
    public static  final float maxVelocity = 1.5f;
    private static final float autoDriveStartLinearAcc = 0.6f * maxAcc;
    private static final float autoDriveEndLinearAcc = 0.6f * maxAcc;
    public static final float maxAutoDriveVel = 0.85f * maxVelocity;
    public static final float maxAccOmega = 1f; //TODO tune all maxes
    public static final float minHeightToOpenFourbar =500;
    private static final float autoDriveStartDOmega = 0.7f * maxAccOmega;
    private static final float autoDriveEndDOmega = 0.1f * maxAccOmega;
    public static final Vector autoDriveLinearAccMaxPoint = new Vector(1, autoDriveStartLinearAcc);
    public static final Vector autoDriveLinearAccMinPoint = new Vector(0, autoDriveEndLinearAcc);

    // Start reducing max dOmega at 1 radian
    public static final Vector autoDriveDOmegaMaxPoint = new Vector(0.3f, autoDriveStartDOmega);
    public static final Vector autoDriveDOmegaMinPoint = new Vector(0, autoDriveEndDOmega);


    public static final float autoDrivePosTolerance = 0.4f;
    public static final float autoDriveVelTolerance = 0.4f;

    public static final float autoDriveAngleTolerance = Angle.degToRad(2.5f);
    public static final float autoDriveOmegaTolerance = Angle.degToRad(10);
    public static final float MaxHeightForFourbarDelay = 1800;
}

package org.firstinspires.ftc.teamcode.robotData;

import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class GlobalData {
    public static RobotState robotState = RobotState.TRAVEL;
    public static boolean hasGamePiece = false;
    public static boolean inAutonomous = false;
    public static float currentTime = 0;
    public static float lastTime = 0;
    public static float deltaTime = 0;
    public static boolean autonomousSide = false; //false = left, true = right
}

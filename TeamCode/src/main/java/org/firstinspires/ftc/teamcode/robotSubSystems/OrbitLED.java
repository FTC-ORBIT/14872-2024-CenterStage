package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class OrbitLED {

    private static RevBlinkinLedDriver blinkin;
    private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static ElapsedTime elapsedTime = new ElapsedTime();
    public static void init(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
    }

    public static void operate(int color) {
        if (color == 1){
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }else if (color == 2){
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        } else if (color == 3) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (color == 4) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        }

        blinkin.setPattern(pattern);
    }
}

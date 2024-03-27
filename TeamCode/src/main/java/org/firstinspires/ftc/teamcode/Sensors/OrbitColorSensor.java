package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitColorSensor {

    public static ColorSensor colorSensor;
    public  static int color = 0;
    public static void  init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
    }


    public static int hasGamePiece() {
            if (colorSensor.argb() == -15985647){
                color = 1;// when white pixel
            } else if (colorSensor.argb() == -100202238) {
                color = 2; // when yellow pixel
            } else if (colorSensor.argb() == -301857278 ) {
                color = 3; // when green pixel
            } else if (colorSensor.argb() == -66779382 || colorSensor.argb() == -66779126) {
                color = 4;// when purple pixel
            }
        return color;
    }

    public void printRGB (Telemetry telemetry){
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("alpha", colorSensor.alpha());
    }

}

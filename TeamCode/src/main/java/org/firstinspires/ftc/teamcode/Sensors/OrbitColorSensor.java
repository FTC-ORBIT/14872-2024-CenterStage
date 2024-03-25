package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitColorSensor {

    public static ColorSensor colorSensor;
    public  static String color = "none";
    public static void  init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
    }


    public static String hasGamePiece() {
            if (colorSensor.argb() == -15985647){
                color = "white";// when white pixel
            } else if (colorSensor.argb() == -100202238) {
                color = "yellow"; // when yellow pixel
            } else if (colorSensor.argb() == -301857278 ) {
                color = "green"; // when green pixel
            } else if (colorSensor.argb() == -66779382 || colorSensor.argb() == -66779126) {
                color = "purple";// when purple pixel
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

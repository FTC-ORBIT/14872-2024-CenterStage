package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitColorSensor {

    public final ColorSensor colorSensor;
    public OrbitColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
    }


    public int hasGamePiece() {
            int color = 0;
            if (colorSensor.red() == 255 && colorSensor.blue() == 255 && colorSensor.green() == 255){
                color = 1;// when white pixel
            } else if (colorSensor.red() == 255 && colorSensor.green() == 255 && colorSensor.blue() == 0) {
                color = 2; // when yellow pixel
            } else if (colorSensor.red() == 50 && colorSensor.green() == 205 && colorSensor.blue() == 50) {
                color = 3; // when green pixel
            } else if (colorSensor.red() == 238 && colorSensor.green() == 130 && colorSensor.blue() == 238) {
                color = 4;
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

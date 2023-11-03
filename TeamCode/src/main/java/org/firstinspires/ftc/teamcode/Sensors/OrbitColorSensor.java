package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitColorSensor {

    private final ColorSensor colorSensor;

    public OrbitColorSensor(HardwareMap hardwareMap, String name) {
        colorSensor = hardwareMap.get(ColorSensor.class, name);
    }


    public boolean hasGamePiece() {
            int colorControlRedCone = 0;
            int colorControlBlueCone = 0;
            float[] currentColors = {colorSensor.red(), colorSensor.green(), colorSensor.blue()};
            for (int i = 0; i < 3; i++){
                if ((currentColors[i] < Constants.blueCone[i] + Constants.colorRange) && (currentColors[i] > Constants.blueCone[i] - Constants.colorRange)){
                    colorControlBlueCone ++;
                }
            }

            for (int i = 0; i < 3; i++){
                if ((currentColors[i] < Constants.redCone[i] + Constants.colorRange) && (currentColors[i] > Constants.redCone[i] - Constants.colorRange)){
                    colorControlRedCone++;
                }
            }
            return (colorControlBlueCone ==3 || colorControlRedCone == 3);
    }

    public void printRGB (Telemetry telemetry){
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
    }

}

package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OrbitDistanceSensor {

  private final DistanceSensor distanceSensor;
  private float lastDistance;

  public OrbitDistanceSensor(HardwareMap hardwareMap) {
    distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
  }

  public float getDistance() {
    return (float) distanceSensor.getDistance(DistanceUnit.INCH);
  }

}

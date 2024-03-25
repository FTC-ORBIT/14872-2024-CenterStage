package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.core.Point;

import java.util.List;

public abstract class AprilTagDetect extends AprilTagProcessor {
    public Point aprilTagCords;
    public int wantedID = 2;
    List<AprilTagDetection> currentDetections = null;
    public int count = 0;
    public int tries = 10;


    protected int getIDfromPosAndColor(PropPosEnum pos, PropColorEnum allianceColor) {


        switch (pos) {
            case LEFT:
                wantedID = 1;
                break;
            case CENTER:
            default:
                wantedID = 2;
                break;
            case RIGHT:
                wantedID = 3;
                break;
        }

        if (allianceColor == PropColorEnum.RED) {
            wantedID += 3;
        }
        return wantedID;
    }

    public Point getAprilTagCords(PropPosEnum pos, PropColorEnum allianceColor) {
        getIDfromPosAndColor(pos, allianceColor);
        count = 0;
        while (count < tries && aprilTagCords == null) {
            currentDetections = getDetections();

            count++;
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            for (AprilTagDetection dtct : currentDetections) {
                if (dtct.metadata != null) {
                    if (dtct.id == wantedID) {
                        // dtct.center is the center in x y cords and not center as a pos
                        aprilTagCords = dtct.center;
                    }
                }
            }
            sleep(5);

        }
        return aprilTagCords;
    }
}


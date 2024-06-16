package org.firstinspires.ftc.teamcode.robotSubSystems.camera.conceptsAndPrototypes;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Webcam2 {

    public static VisionPortal ActivePortal;
    private WebcamName ActiveCamera;
    private VisionPortal portal;
    private VisionPortal portal2;

    public void init(WebcamName webcam, WebcamName webcam2, VisionProcessor... processors) {
        portal = portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .addProcessors(processors)
                .build();
        portal2 = portal = new VisionPortal.Builder()
                .setCamera(webcam2)
                .setCameraResolution(new Size(640, 480))
                .addProcessors(processors)
                .build();
        portal2.close();
    }

    public VisionPortal switchCameraNotProteceted(WebcamName camera) {
        if(camera.equals(portal.getActiveCamera())) {
            portal.close();
            while(portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            }
            portal2.resumeStreaming();
            ActiveCamera = portal2.getActiveCamera();
            ActivePortal = portal2;
            return portal2;
        }
        else {
            portal2.close();
            while(portal2.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            }
            portal.resumeStreaming();
            ActiveCamera = portal.getActiveCamera();
            ActivePortal = portal;
            return portal;
        }
    }

    public VisionPortal switchCamera(WebcamName camera) {
        VisionPortal vportal = switchCameraNotProteceted(camera);
        while(vportal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }
        return vportal;
    }

    public VisionPortal switchCamera() { //Switches camera and returns the portal(waits untill the camera is streaming before returning the portal)
        if(ActiveCamera == portal.getActiveCamera()) {
            return switchCamera(portal2.getActiveCamera());
        }
        else {
            return switchCamera(portal.getActiveCamera());
        }
    }

    public VisionPortal switchCameraNotProteceted() { //Switches camera and returns it instantly
        if(ActiveCamera == portal.getActiveCamera()) {
            return switchCameraNotProteceted(portal2.getActiveCamera());
        }
        else {
            return switchCameraNotProteceted(portal.getActiveCamera());
        }
    }


    public VisionPortal getWebcam2() {
        if(ActiveCamera == portal.getActiveCamera()) {
            return portal2;
        }
        else {
            return portal;
        }
    }
}

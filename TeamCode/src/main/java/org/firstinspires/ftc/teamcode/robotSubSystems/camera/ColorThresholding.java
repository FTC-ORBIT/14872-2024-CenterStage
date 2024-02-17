package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;



    @TeleOp
    public class ColorThresholding extends LinearOpMode
    {
        OpenCvWebcam webcam;

        @Override
        public void runOpMode()
        {
            /*
             * Instantiate an OpenCvCamera object for the camera we'll be using.
             * In this sample, we're using a webcam. Note that you will need to
             * make sure you have added the webcam to your configuration file and
             * adjusted the name here to match what you named it in said config file.
             *
             * We pass it the view that we wish to use for camera monitor (on
             * the RC phone). If no camera monitor is desired, use the alternate
             * single-parameter constructor instead (commented out below)
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View
            //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
            /*
             * Specify the image processing pipeline we wish to invoke upon receipt
             * of a frame from the camera. Note that switching pipelines on-the-fly
             * (while a streaming session is in flight) *IS* supported.
             */
            webcam.setPipeline(new rectangle_thresholder_pipeline());

            /*
             * Open the connection to the camera device. New in v1.4.0 is the ability
             * to open the camera asynchronously, and this is now the recommended way
             * to do it. The benefits of opening async include faster init time, and
             * better behavior when pressing stop during init (i.e. less of a chance
             * of tripping the stuck watchdog)
             *
             * If you really want to open synchronously, the old method is still available.
             */
            webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    /*
                     * Tell the webcam to start streaming images to us! Note that you must make sure
                     * the resolution you specify is supported by the camera. If it is not, an exception
                     * will be thrown.
                     *
                     * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                     * supports streaming from the webcam in the uncompressed YUV image format. This means
                     * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                     * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                     *
                     * Also, we specify the rotation that the webcam is used in. This is so that the image
                     * from the camera sensor can be rotated such that it is always displayed with the image upright.
                     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                     * away from the user.
                     */
                       webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                 //   webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            telemetry.addLine("Waiting for start");
            telemetry.update();

            /*
             * Wait for the user to press start on the Driver Station
             */
            waitForStart();

            while (opModeIsActive())
            {
                /*
                 * Send some stats to the telemetry
                 */
//                telemetry.addData("Frame Count", webcam.getFrameCount());
//                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
//                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
//                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
//                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
//                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

                /*
                 * NOTE: stopping the stream from the camera early (before the end of the OpMode
                 * when it will be automatically stopped for you) *IS* supported. The "if" statement
                 * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
                 */
                if(gamepad1.a)
                {
                    /*
                     * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                     * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                     * if the reason you wish to stop the stream early is to switch use of the camera
                     * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                     * (commented out below), because according to the Android Camera API documentation:
                     *         "Your application should only have one Camera object active at a time for
                     *          a particular hardware camera."
                     *
                     * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                     * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                     *
                     * NB2: if you are stopping the camera stream to simply save some processing power
                     * (or battery power) for a short while when you do not need your vision pipeline,
                     * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                     * it the next time you wish to activate your vision pipeline, which can take a bit of
                     * time. Of course, this comment is irrelevant in light of the use case described in
                     * the above "important note".
                     */
                    webcam.stopStreaming();
                    //webcam.closeCameraDevice();
                }

                /*
                 * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
                 * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
                 * anyway). Of course in a real OpMode you will likely not want to do this.
                 */
                sleep(100);
            }
        }
        public class rectangle_thresholder_pipeline extends OpenCvPipeline {
            private String location = "nothing"; // output
        //    public Scalar lower = new Scalar(200, 15, 20); // HSV threshold bounds
        //    public Scalar upper = new Scalar(240, 100, 60);
        public Scalar lower = new Scalar(0, 0, 210); // HSV threshold bounds
            public Scalar upper = new Scalar(255, 100, 255);

            private Mat hsvMat = new Mat(); // converted image
            private Mat binaryMat = new Mat(); // image analyzed after thresholding
            private Mat maskedInputMat = new Mat();

            // Rectangle regions to be scanned
            private Point topLeft1 = new Point(0, 0), bottomRight1 = new Point(239, 159);
            private Point topLeft2 = new Point(0, 160), bottomRight2 = new Point(239, 319);

            public rectangle_thresholder_pipeline() {

            }

            @Override
            public Mat processFrame(Mat input) {
                // Convert from BGR to HSV
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
                Core.inRange(hsvMat, lower, upper, binaryMat);


                // Scan both rectangle regions, keeping track of how many
                // pixels meet the threshold value, indicated by the color white
                // in the binary image
                double b1 = 0, b2 = 0;
                // process the pixel value for each rectangle  (255 = W, 0 = B)
                for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
                    for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                        if (binaryMat.get(i, j)[0] >= 180) {
                            b1++;
                        }
                    }
                }

                for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
                    for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                        if (binaryMat.get(i, j)[0] >=180) {
                            b2++;
                        }
                    }
                }
                // Determine object location
                telemetry.addData("Blue pixels on the left", b1);
                telemetry.addData("Blue pixels on the right", b2);
                if (b1 > b2) {
                    location = "1";
                    telemetry.addLine("The blue object is on:     LEFT");

                } else if (b1 < b2) {
                    location = "2";
                    telemetry.addLine("The blue object is on:     RIGHT");
                }
                telemetry.update();
                return binaryMat;
            }

            public String getLocation() {
                return location;
            }
        }
    }


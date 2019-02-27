package org.firstinspires.ftc.teamcode.seasons.roverruckus.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public abstract class VufTFLiteHandler extends LinearOpMode {
    //Params:
    public static final String TAG = "Vuforia Navigation Sample";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    OpenGLMatrix lastLocation = null;

    private static final String VUFORIA_KEY = "Acu/r3//////AAABmUCiz2XQhk/dnQ69OhvvEQ6EIGB68o2" +
            "EKHda/lIM/by4dkuRcKfh8orF1OUCT5ySk1MRZ94z6aTWHSZjAqkLsmFEERep2QYULUfdAIg5NY54bI0v" +
            "MtmV1ulwLZYty21pM43lYETu/aoW6zT6vg5eYjPSn8eN9tB0Hdio6tRMaZN6iA9J6fRYHbty/cv2WVMjq" +
            "KD+sZE+UByTrY1l0s56dySlKMRbJqA5wJgqzjunz7Xh+lyNmAcKr1pRKlRH99PGnKyvEcUA1nhxXL0qKX" +
            "NzHgsi2Z7hJzpd89e3/EAouMY6foY+AhnZSlcyv+OKCSOa8z2MYbTWG/zD5R5BXpC1yzmRUZwdBIi74wY9rfZLR8Vv";

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public int goldMineralX;

    public String goldMineralPosition = "NotSetYet";
    public int numMineralsDetected;

    private boolean targetVisible;
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    WebcamName webcamName;


    final float mmPerInch = 25.4f;
    final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    VuforiaTrackables targetsRoverRuckus;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    public void getTensorFlowData() {
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
//            telemetry.addData("# Objects Detected: ", x);
        if (updatedRecognitions != null) {
//            telemetry.addData("# Object Detected", updatedRecognitions.size());
//                x = updatedRecognitions.size();
            numMineralsDetected = updatedRecognitions.size();



            if (updatedRecognitions.size() == 1 ||updatedRecognitions.size() == 2 || updatedRecognitions.size() ==3 ) {
                goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {


                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();

//                        telemetry.addData("PixelsToGoldLeftSide", recognition.getLeft());
//                        telemetry.addData("PixelsToGoldRightSide", recognition.getRight());
//
//                        telemetry.addData("GoldMineralPixelWidth", recognition.getRight() - recognition.getLeft());

                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
//                telemetry.addData("goldMineralX", goldMineralX);
//                telemetry.addData("silverMineralX", silverMineral1X);
//                telemetry.addData("silverMineralX2", silverMineral2X);

                //if see 3 logic
                if (updatedRecognitions.size() == 3) {
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                        telemetry.addData("Gold Mineral Position", "Left");
                            goldMineralPosition = "Left";
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                        telemetry.addData("Gold Mineral Position", "Right");
                            goldMineralPosition = "Right";
                        } else {
//                        telemetry.addData("Gold Mineral Position", "Center");
                            goldMineralPosition = "Center";
                        }
                    }
                } else {   //if see 2 logic
                    if (updatedRecognitions.size() == 2) {
                        if (goldMineralX != -1) {//if see gold mineral out of 2 seen
                            if (goldMineralX < silverMineral1X) {
                                goldMineralPosition = "Left";
                            } else {
                                goldMineralPosition = "Center";
                            }
                        } else {
                            goldMineralPosition = "Right";
                        }
                    }
                }
            }
                else
                {
                    goldMineralPosition = "notDetected";
                }
            }
        }

    public void stopTensorFlow() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void getVuforiaData() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
    }

    //This method will init and activate all methods that are needed for both Vuforia and TFLite
    public void initAll() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);
        targetsRoverRuckus.activate();
        /** For convenience, gather together all the trackable objects in one easily-iterable collection */

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        vuforia.enableConvertFrameToBitmap();

    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public String getGoldMineralPosition() {
        return goldMineralPosition;
    }


}


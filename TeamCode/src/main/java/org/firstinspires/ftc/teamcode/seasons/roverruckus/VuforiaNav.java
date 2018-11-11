package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaNav {
    private static String key = "ATI4rJn/////AAAAGaU649WKjUPNjEAS8RHKRjFL+3htvDbOq/i5kIheS3" +
            "6mg04zehxVYIKQacMQbsHteW8MaFqibM5CTPJkObcAnnIe+Yt8uA2E288cN1g2LRuu6OmJWUgUNrfH9Oe" +
            "p/MDDh8/mOWD2osAziNUsh7xYdb2FH6VmGFomR8Whb1i+3t5ilAGd0mIOd6OFFSA8IcRxcw9EfE0SIFmg" +
            "SXwi05SMU5CkUtnidIBpC+w6wfNp2BLL863XgaZjfpsNz57TaKqgxuy/HBjec0uvS2JQ/vBVeKV72FGVv" +
            "SLJlPh/nQZ964O/5VpNbHUmZYhyKYQgTvP2HuUZ+azjY/WGUDTOCB2ZxZj9wSmX5iCY7k9rI0cMNIri";
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;
    public static final String TAG = "Vuforia VuMark Sample";
    private HardwareMap hardwareMap = null;
    public VuforiaLocalizer vuforia;

    public VuforiaTrackables ruckusTrackables;
    public VuforiaTrackable ruckusTrackable;

    public VuforiaNav(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = key;

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        ruckusTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        ruckusTrackable = ruckusTrackables.get(0);
        ruckusTrackable.setName("ruckusVuMarkTemplate"); // can help in debugging; otherwise not necessary
        parameters.vuforiaLicenseKey =
                "ATI4rJn/////AAAAGaU649WKjUPNjEAS8RHKRjFL+3htvDbOq/i5kIheS3" +
                        "6mg04zehxVYIKQacMQbsHteW8MaFqibM5CTPJkObcAnnIe+Yt8uA2E288cN1g2LRuu6OmJWUgUNrfH9Oe" +
                        "p/MDDh8/mOWD2osAziNUsh7xYdb2FH6VmGFomR8Whb1i+3t5ilAGd0mIOd6OFFSA8IcRxcw9EfE0SIFmg" +
                        "SXwi05SMU5CkUtnidIBpC+w6wfNp2BLL863XgaZjfpsNz57TaKqgxuy/HBjec0uvS2JQ/vBVeKV72FGVv" +
                        "SLJlPh/nQZ964O/5VpNbHUmZYhyKYQgTvP2HuUZ+azjY/WGUDTOCB2ZxZj9wSmX5iCY7k9rI0cMNIri";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

    }

    public void activate() {
        ruckusTrackables.activate();
    }
}


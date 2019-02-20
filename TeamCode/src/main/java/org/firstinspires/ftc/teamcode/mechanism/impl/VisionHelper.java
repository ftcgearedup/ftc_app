package org.firstinspires.ftc.teamcode.mechanism.impl;

import android.app.Activity;

import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;


/**
 * A helper class that initializes Vuforia and is a wrapper for {@link VuforiaLocalizer}.
 * Using Vuforia,
 */

public class VisionHelper implements IMechanism {
    private static final String LOG_TAG = "VisionHelper";

    private Activity activity;
    private VuforiaLocalizer vuforia;

    /**
     * Construct a new instance of this class with a reference to the robot and the
     * {@link VuforiaLocalizer} currently in use.
     *
     * @param robot the robot utilizing this object
     */
    public VisionHelper(Robot robot) {
        this.activity = (Activity)robot.getCurrentOpMode().hardwareMap.appContext;
    }

    /**
     * Initialize Vuforia with the specified {@code cameraDirection}.
     * On a ZTE Speed phone, this method blocks for a few seconds as determined by minimal testing.
     *
     * @param cameraDirection the desired camera direction to use with Vuforia
     */
    public void initializeVuforia(VuforiaLocalizer.CameraDirection cameraDirection) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "ATI4rJn/////AAAAGaU649WKjUPNjEAS8RHKRjFL+3htvDbOq/i5kIheS3" +
                "6mg04zehxVYIKQacMQbsHteW8MaFqibM5CTPJkObcAnnIe+Yt8uA2E288cN1g2LRuu6OmJWUgUNrfH9Oe" +
                "p/MDDh8/mOWD2osAziNUsh7xYdb2FH6VmGFomR8Whb1i+3t5ilAGd0mIOd6OFFSA8IcRxcw9EfE0SIFmg" +
                "SXwi05SMU5CkUtnidIBpC+w6wfNp2BLL863XgaZjfpsNz57TaKqgxuy/HBjec0uvS2JQ/vBVeKV72FGVv" +
                "SLJlPh/nQZ964O/5VpNbHUmZYhyKYQgTvP2HuUZ+azjY/WGUDTOCB2ZxZj9wSmX5iCY7k9rI0cMNIri";

        parameters.cameraDirection = cameraDirection;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }





    /**
     * Get the {@link VuforiaLocalizer} for robot algorithms to use.
     *
     * @return the vuforia localizer instance this class wraps around
     */
    public VuforiaLocalizer getVuforia() {
         return vuforia;
     }
}

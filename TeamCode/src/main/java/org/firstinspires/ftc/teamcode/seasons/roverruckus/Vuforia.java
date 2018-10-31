package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;

/**
 * This algorithm is used to scan the VuMark in autonomous.
 */

public class Vuforia {

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables Trackables;
    private VuforiaTrackable imageTarget;

    /**
     * Create a new VuMarkScanAlgorithm.
     *
     * @param robot the robot using this algorithm
     * @param visionHelper the vision helper instance required to access vision
     */
    public Vuforia(Robot robot, VisionHelper visionHelper) {
        this.vuforia = visionHelper.getVuforia();

        this.Trackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        this.imageTarget = Trackables.get(0);
        imageTarget.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    /**
     * Activate active detection of VuMarks.
     * After calling this method, the Vuforia will continually search for the VuMark.
     */
    public void activate() {
        Trackables.activate();
    }

    /**
     * Deactivate active detection of VuMarks.
     * After calling this method, the Vuforia will stop continually search for the VuMark.
     */
    public void deactivate() {
       Trackables.deactivate();
    }

    /**
     * Return the currently detected VuMark.
     *
     * @see RelicRecoveryVuMark the enumeration type for the three VuMark types
     * @return the VuMark enumeration type
     */
    public RelicRecoveryVuMark detect() {
        return RelicRecoveryVuMark.from(imageTarget);
    }
}

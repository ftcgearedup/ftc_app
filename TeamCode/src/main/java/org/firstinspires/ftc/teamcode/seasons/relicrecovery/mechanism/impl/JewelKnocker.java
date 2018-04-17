package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * This class is responsible for the control of the jewel knocker, which is used
 * to knock a jewel off of the jewel platform.
 */

public class JewelKnocker implements IMechanism {
    private Servo knockerServo;
    private Servo armServo;

    private ColorSensor jewelColorSensor;

    private static JSONConfigOptions optionsMap;

    private OpMode opMode;

    ElapsedTime timer = new ElapsedTime();

    private final int SECOND_JEWEL_ARM_DELAY_MS;
    private final int FIRST_JEWEL_ARM_DELAY_MS;
    private final int JEWEL_BLUE_LEVEL = 30;
    private final int JEWEL_RED_LEVEL = 50;

    /**
     * Construct a new {@link JewelKnocker} instance.
     *
     * @param robot the robot to use this JewelKnocker object
     */
    public JewelKnocker(Robot robot) {
        this.opMode = robot.getCurrentOpMode();
        HardwareMap hwMap = opMode.hardwareMap;

        optionsMap = ((RelicRecoveryRobot)robot).getOptionsMap();

        SECOND_JEWEL_ARM_DELAY_MS = optionsMap.retrieveAsInt("jewelKnockerDelayMS");
        FIRST_JEWEL_ARM_DELAY_MS = optionsMap.retrieveAsInt("jewelKnockerScanMS");

        this.armServo = hwMap.servo.get("js");
        this.knockerServo = hwMap.servo.get("rs");

        this.jewelColorSensor = hwMap.colorSensor.get("jcs");
        jewelColorSensor.enableLed(true);

        // retract arm and set center rotation position in initialization
        retractArm();
        leftRotation();
    }

    /**
     * This method implements an algorithm to knock the appropriate jewel off of the jewel platform.
     *
     * @param isRedAlliance whether the robot is on the red alliance. This is used to determine
     *                      if the robot should knock the blue jewel off of the platform.
     */
    public void knockJewel(boolean isRedAlliance) {
        LinearOpMode linearOpMode;
        if(opMode instanceof LinearOpMode) {
            linearOpMode = (LinearOpMode)opMode;

            centerRotation();

            // extend servo arm
            extendArm();

            timer.reset();
            if(timer.milliseconds() <= FIRST_JEWEL_ARM_DELAY_MS){
                if(isRedAlliance) {
                    if(isJewelBlue()) {
                        leftRotation();
                    } else {
                        rightRotation();
                    }
                } else {
                    if(isJewelBlue()) {
                        rightRotation();
                    } else {
                        leftRotation();
                    }
                }

                // rotate left or right
                timer.reset();
                if(timer.milliseconds() <= SECOND_JEWEL_ARM_DELAY_MS){
                    // retract servo arm
                    retractArm();

                    timer.reset();
                    if(timer.milliseconds() <= SECOND_JEWEL_ARM_DELAY_MS){
                        linearOpMode.idle();
                    }
                }
            }
        }
    }

    /**
     * Retracts Jewel Arm
     */
    public void retractArm() {
        armServo.setPosition(optionsMap.retrieveAsDouble("jewelKnockerRetractPosition"));
    }

    /**
     * Extends Jewel Arm
     */
    public void extendArm() {
        armServo.setPosition(optionsMap.retrieveAsDouble("jewelKnockerExtendPosition"));
    }

    /**
     * Rotate the knocker servo to the left
     */
    public void leftRotation() {
        knockerServo.setPosition(optionsMap.retrieveAsDouble("jewelKnockerLeftRotation"));
    }

    /**
     * Rotate the knocker servo to the center position
     */
    public void centerRotation() {
        knockerServo.setPosition(optionsMap.retrieveAsDouble("jewelKnockerCenterRotation"));
    }

    /**
     * Rotate the knocker servo to the right
     */
    public void rightRotation() {
        knockerServo.setPosition(optionsMap.retrieveAsDouble("jewelKnockerRightRotation"));
    }

    /**
     * @return the red level detected
     */
    public int getRed(){
        return jewelColorSensor.red();
    }

    /**
     * @return the blue level detected
     */
    public int getBlue(){
        return jewelColorSensor.blue();
    }

    private boolean isJewelRed(){
        return getRed() > getBlue();
    }

    private boolean isJewelBlue(){
        return getBlue() > getRed();
    }
}

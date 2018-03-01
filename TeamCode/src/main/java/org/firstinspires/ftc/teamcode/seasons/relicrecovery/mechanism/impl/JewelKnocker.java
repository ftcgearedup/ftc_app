package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

import java.io.File;

/**
 * This class is responsible for the control of the jewel knocker, which is used
 * to knock a jewel off of the jewel platform.
 */

public class JewelKnocker implements IMechanism {
    private Servo knockerServo;
    private Servo armServo;

    private ColorSensor jewelColorSensor;

    private OpMode opMode;

    private static final int JEWEL_ARM_DELAY_MS = 500;

    /**
     * Construct a new {@link JewelKnocker} instance.
     *
     * @param robot the robot to use this JewelKnocker object
     */
    public JewelKnocker(Robot robot) {
        this.opMode = robot.getCurrentOpMode();
        HardwareMap hwMap = opMode.hardwareMap;

        this.armServo = hwMap.servo.get("js");
        this.knockerServo = hwMap.servo.get("rs");

        this.jewelColorSensor = hwMap.colorSensor.get("jcs");
        jewelColorSensor.enableLed(true);

        // retract arm and set center rotation position in initialization
        retractArm();
        rightRotation();
    }

    /**
     * This method implements an algorithm to knock the appropriate jewel off of the jewel platform.
     *
     * @param isRedAlliance whether the robot is on the red alliance. This is used to determine
     *                      if the robot should knock the blue jewel off of the platform.
     */
    public void knockJewel(boolean isRedAlliance) {
        ElapsedTime timer = new ElapsedTime();

        LinearOpMode linearOpMode;
        if(opMode instanceof LinearOpMode) {
            linearOpMode = (LinearOpMode)opMode;

            centerRotation();

            // extend servo arm
            timer.reset();
            while(linearOpMode.opModeIsActive() && timer.milliseconds() < JEWEL_ARM_DELAY_MS) {
                extendArm();
            }

            // rotate left or right
            timer.reset();
            while(linearOpMode.opModeIsActive() && timer.milliseconds() < JEWEL_ARM_DELAY_MS) {
                if (isRedAlliance && isJewelBlue()) {
                    leftRotation();
                } else {
                    rightRotation();
                }
            }

            // retract servo arm
            retractArm();
        }
    }

    /**
     * Retracts Jewel Arm
     */
    public void retractArm() {
        armServo.setPosition(0.3);
    }

    /**
     * Extends Jewel Arm
     */
    public void extendArm() {
        armServo.setPosition(0.85);
    }

    /**
     * Rotate the knocker servo to the left
     */
    public void leftRotation() {
        knockerServo.setPosition(0);
    }

    /**
     * Rotate the knocker servo to the center position
     */
    public void centerRotation() {
        knockerServo.setPosition(0.5);
    }

    /**
     * Rotate the knocker servo to the right
     */
    public void rightRotation() {
        knockerServo.setPosition(1.0);
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

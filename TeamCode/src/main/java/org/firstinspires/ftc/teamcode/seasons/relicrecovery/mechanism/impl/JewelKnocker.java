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
 *
 */

public class JewelKnocker implements IMechanism {
    private final JSONConfigOptions optionsMap = new JSONConfigOptions("options.json");

    private Servo knockerServo;
    private Servo armServo;

    private ColorSensor jewelColorSensor;

    private OpMode opMode;

    private final int JEWEL_ARM_DELAY_MS;

    private int retractPos;
    private int extendPos;
    private int leftRotationPos;
    private int centerRotationPos;
    private int rightRotationPos;



    public JewelKnocker(Robot robot) {
        this.opMode = robot.getCurrentOpMode();
        HardwareMap hwMap = opMode.hardwareMap;

        JEWEL_ARM_DELAY_MS = optionsMap.retrieveAsInt("jewelKnockerDelayMS");

        retractPos = optionsMap.retrieveAsInt("jewelKnockerRetractPosition");
        extendPos = optionsMap.retrieveAsInt("jewelKnockerExtendPosition");
        leftRotationPos = optionsMap.retrieveAsInt("jewelKnockerLeftRotation");
        centerRotationPos = optionsMap.retrieveAsInt("jewelKnockerCenterRotation");
        rightRotationPos = optionsMap.retrieveAsInt("jewelKnockerRightRotation");

        this.armServo = hwMap.servo.get("js");
        this.knockerServo = hwMap.servo.get("rs");

        this.jewelColorSensor = hwMap.colorSensor.get("jcs");
        jewelColorSensor.enableLed(true);

        // retract arm and set center rotation position in initialization
        retractArm();
        rightRotation();
    }

    /**
     * @param isRedAlliance
     */
    public void knockJewel(boolean isRedAlliance) {
        ElapsedTime timer = new ElapsedTime();

        LinearOpMode linearOpMode;
        if(opMode instanceof LinearOpMode) {
            linearOpMode = (LinearOpMode)opMode;

            centerRotation();

            while(linearOpMode.opModeIsActive() && timer.milliseconds() < JEWEL_ARM_DELAY_MS) {
                extendArm();
            }

            if(isRedAlliance && isJewelBlue()) {
                leftRotation();
            } else {
                rightRotation();
            }
        }
    }

    /**
     * Retracts Jewel Arm
     */
    public void retractArm() {
        armServo.setPosition(retractPos);
    }

    /**
     * Extends Jewel Arm
     */
    public void extendArm() {
        armServo.setPosition(extendPos);
    }

    /**
     *
     */
    public void leftRotation() {
        knockerServo.setPosition(leftRotationPos);
    }

    /**
     *
     */
    public void centerRotation() {
        knockerServo.setPosition(centerRotationPos);
    }

    /**
     *
     */
    public void rightRotation() {
        knockerServo.setPosition(rightRotationPos);
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

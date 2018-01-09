package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;

/**
 *
 */

public class JewelKnocker implements IMechanism {
    private Servo arm;

    private ColorSensor jewelColorSensor;


    public JewelKnocker(Robot robot) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;

        this.arm = hwMap.servo.get("jks");

        this.jewelColorSensor = hwMap.colorSensor.get("jcs");
        jewelColorSensor.enableLed(true);
    }
    /**
     * Retracts Jewel Arm
     */
    public void retractArm() {
        arm.setPosition(0.2);
    }
    /**
     * Extends Jewel Arm
     */
    public void extendArm() {
        arm.setPosition(1.0);
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
    /**
     * Detects if the color detected is more red than blue.
     *
     * @return true if more red, false if more blue
     */
    public boolean isJewelRed(){
        int blueLevel = getBlue();
        int redLevel = getRed();
        boolean isRed = false;

        if(redLevel > blueLevel){
            isRed= true;
        } else {
            isRed= false;
        }

        return isRed;
    }
    /**
     * Detects if the color detected is more blue than red.
     *
     * @return true if more blue, false if more red
     */
    public boolean isJewelBlue(){
        int blueLevel = getBlue();
        int redLevel = getRed();
        boolean isBlue = false;

        if(blueLevel > redLevel){
            isBlue= true;
        } else {
            isBlue= false;
        }

        return isBlue;
    }
}

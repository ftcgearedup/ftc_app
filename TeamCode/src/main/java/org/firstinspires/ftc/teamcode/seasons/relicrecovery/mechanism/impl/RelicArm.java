package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;

/**
 *
 */

public class RelicArm implements IMechanism {
    private DcMotor armMain;

    private Servo gripper;
    private CRServo armEx;

    private boolean isClosed = false;


    public RelicArm(Robot robot) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;
        this.armMain = hwMap.dcMotor.get("ram");
        this.armMain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.armMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gripper = hwMap.servo.get("rag");
        this.armEx = hwMap.crservo.get("rae");
        this.armEx.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    /**
     * Opens the Relic Arm Gripper
     */
    public void openGrip() {
        gripper.setPosition(0);
        isClosed = false;
    }
    /**
     * Closes the Relic Arm Gripper
     */
    public void closeGrip() {
        gripper.setPosition(1);
        isClosed = true;
    }
    /**
     * Sets power of the Main Relic Arm
     *
     * @param power The power you want to set the Main Arm to move at
     */
    public void setArmMainPower(double power) {
        armMain.setPower(power);
    }
    /**
     * Sets power of the Relic Arm Extension
     *
     * @param power The power you want to set the extension to move at
     */
    public void setArmExtensionPower(double power) {
        armEx.setPower(power);
    }
    /**
     * @return if the Relic Gripper is Closed or not (boolean)
     */
    public boolean isGripClosed(){
        return isClosed;
    }
    /**
     * Initializes Relic Arm
     */
    public void initializeRelicArm(){
        armEx.setPower(0);
        armMain.setPower(0);
        openGrip();
    }
}
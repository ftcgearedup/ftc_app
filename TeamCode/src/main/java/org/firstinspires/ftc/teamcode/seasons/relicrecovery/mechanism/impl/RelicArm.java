package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;

/**
 *
 */

public class RelicArm implements IMechanism {
    private DcMotor armMotor;

    private Servo gripperServo;
    private Servo armRotationServo;

    private boolean isClosed = false;

    public RelicArm(Robot robot) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;

        armMotor = hwMap.dcMotor.get("ram");
        gripperServo = hwMap.servo.get("rag");
        armRotationServo = hwMap.servo.get("rar");

        // brake and run using encoder for
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initially set the gripper servo position to open
        openGrip();
    }

    /**
     * Raises the relic arm rotation servo
     */
    public void raiseArmRotation() {
        armRotationServo.setPosition(1.0);
    }

    /**
     * Lowers the relic arm rotation servo
     */
    public void lowerArmRotation() {
        armRotationServo.setPosition(0.3);
    }

    public void initializeArmRotationPosition() {
        armRotationServo.setPosition(0);
    }

    /**
     * Opens the relic arm gripper
     */
    public void openGrip() {
        gripperServo.setPosition(0);
        isClosed = false;
    }

    /**
     * Closes the relic arm gripper
     */
    public void closeGrip() {
        gripperServo.setPosition(1.0);
        isClosed = true;
    }

    /**
     * Sets power of the main relic arm motor
     *
     * @param power The power you want to set the Main Arm to move at
     */
    public void setArmMainPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * @return if the Relic Gripper is Closed or not (boolean)
     */
    public boolean isGripClosed(){
        return isClosed;
    }
}
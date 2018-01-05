package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;

/**
 * This class represents the glyph intake system on the robot.
 */

public class Intake implements IMechanism {
    private Servo leftArmServo, rightArmServo;
    private CRServo leftWheelServo, rightWheelServo;
    private DcMotor intakeLinkage;
    private boolean isLinkageMotorRunning = false;
    private boolean isLinkageClosed;
    private ElapsedTime linkageTimer;

    private final int LINKAGE_CLOSED_ROTATION_DEGREES;

    /**
     * Construct a new {@link Intake} with a reference to the utilizing robot.
     *
     * @param robot the robot using this intake
     */
    public Intake(Robot robot) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;

        this.leftArmServo = hwMap.servo.get("al");
        this.rightArmServo = hwMap.servo.get("ar");

        this.leftWheelServo = hwMap.crservo.get("wl");
        this.rightWheelServo = hwMap.crservo.get("wr");
        this.intakeLinkage = hwMap.dcMotor.get("link");
        this.linkageTimer = new ElapsedTime();

        LINKAGE_CLOSED_ROTATION_DEGREES = (int) (intakeLinkage.getMotorType().getTicksPerRev() / 2.2);

        intakeLinkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Lower the intake servo motors.
     */
    public void lowerIntake() {
        leftArmServo.setPosition(0.05);
        rightArmServo.setPosition(0.9);
    }
    /**
     * Opens the Intake Linkage
     */
    public void openLinkage() {
        if(isLinkageClosed) {
            if(!isLinkageMotorRunning) {
                intakeLinkage.setTargetPosition(0);
                intakeLinkage.setPower(1);
                isLinkageMotorRunning = true;
            } else if (!intakeLinkage.isBusy()) {
                isLinkageClosed = false;
                isLinkageMotorRunning = false;
                intakeLinkage.setPower(0);
            }
        }
    }

    /**
     * Closes the Intake Linkage
     */
    public void closeLinkage() {
        if(!isLinkageClosed) {
            if(!isLinkageMotorRunning) {
                intakeLinkage.setTargetPosition(LINKAGE_CLOSED_ROTATION_DEGREES);
                intakeLinkage.setPower(1);
                isLinkageMotorRunning = true;
            } else if (!intakeLinkage.isBusy()) {
                isLinkageClosed = true;
                isLinkageMotorRunning = false;
                intakeLinkage.setPower(0);
            }
        }
    }

    /**
     * Raise the intake servo motors.
     */
    public void raiseIntake() {

        leftArmServo.setPosition(0.90);

        leftArmServo.setPosition(0.9);

        rightArmServo.setPosition(0.05);
    }

    /**
     * Set the power of the intake wheels.
     *
     * @param power any value in the range of 1.0 to -1.0.
     *              Negative values run the intake in reverse.
     */
    public void setIntakePower(double power) {
        leftWheelServo.setPower(power);

        // this servo needs to run in the opposite direction:
        rightWheelServo.setPower(-power);
    }
    /**
     * @return if the Linkage is closed
     */
    public boolean isLinkClosed() {
        return isLinkageClosed;
    }

}

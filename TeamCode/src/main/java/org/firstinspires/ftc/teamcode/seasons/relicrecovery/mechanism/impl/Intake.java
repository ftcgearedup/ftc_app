package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        if(!isLinkageMotorRunning){
            intakeLinkage.setTargetPosition((int) -(intakeLinkage.getMotorType().getTicksPerRev() / 2));
            intakeLinkage.setPower(0.5);
            isLinkageMotorRunning = true;
        } else if(!intakeLinkage.isBusy()) {
            isLinkageClosed = false;
            isLinkageMotorRunning = false;
            intakeLinkage.setPower(0);
        }

    }
    /**
     * Closes the Intake Linkage
     */
    public void closeLinkage(){
        if(!isLinkageMotorRunning){
            intakeLinkage.setTargetPosition((int) (intakeLinkage.getMotorType().getTicksPerRev() / 2));
            intakeLinkage.setPower(0.5);
            isLinkageMotorRunning = true;
        } else if(!intakeLinkage.isBusy()) {
            isLinkageClosed = true;
            isLinkageMotorRunning = false;
            intakeLinkage.setPower(0);
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

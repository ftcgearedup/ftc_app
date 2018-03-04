package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * The relic arm class is represents the relic arm mechanism on the robot.
 */

public class RelicArm implements IMechanism {
    private DcMotor armMotor;

    private Servo gripperServo;
    private Servo armRotationServo;

    private boolean isClosed = false;

    private OpMode opMode;

    private JSONConfigOptions optionsMap;

    private final int ARM_MOTOR_MAX_POSITION;

    /**
     * Create a new {@link RelicArm} instance.
     *
     * @param robot the robot using this relic arm object
     */
    public RelicArm(Robot robot) {
        this.opMode = robot.getCurrentOpMode();
        HardwareMap hwMap = opMode.hardwareMap;

        this.optionsMap = ((RelicRecoveryRobot)robot).getOptionsMap();

        ARM_MOTOR_MAX_POSITION = optionsMap.retrieveAsInt("relicArmMotorMaxPosition");

        armMotor = hwMap.dcMotor.get("ram");
        gripperServo = hwMap.servo.get("rag");
        armRotationServo = hwMap.servo.get("rar");

        // brake and run using encoder for
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initially set the gripper servo position to 0.5
        closeGripper();
        initializeArmRotationPosition();
    }

    private void stepServoPosition(double position, double amount) {
        double currentPosition = armRotationServo.getPosition();

        opMode.telemetry.addData("arm rotation servo", currentPosition);
        opMode.telemetry.update();

        if(currentPosition < position) {
            armRotationServo.setPosition(currentPosition + amount);
        } else if(currentPosition > position) {
            armRotationServo.setPosition(currentPosition - amount);
        }
    }

    public int getArmMotorPosition() {
        return armMotor.getCurrentPosition();
    }

    /**
     * Raises the relic arm rotation servo
     */
    public void raiseArmRotation() {
        stepServoPosition(0.2, 0.02);
    }

    /**
     * Lowers the relic arm rotation servo
     */
    public void lowerArmRotation() {
        stepServoPosition(1.0, 0.02);
    }

    private void initializeArmRotationPosition() {
        armRotationServo.setPosition(0.02);
    }

    /**
     * Opens the relic arm gripper
     */
    public void openGripper() {
        gripperServo.setPosition(0.5);
        isClosed = false;
    }

    /**
     * Closes the relic arm gripper
     */
    public void closeGripper() {
        gripperServo.setPosition(1.0);
        isClosed = true;
    }

    /**
     * Sets power of the main relic arm motor
     *
     * @param power The power you want to set the Main Arm to move at
     */
    public void setArmMainPower(double power) {
        if(power < 0 &&
                (armMotor.getCurrentPosition() <= ARM_MOTOR_MAX_POSITION
                        || armMotor.getCurrentPosition() >= 0)) {
            power = 0;
        }

        armMotor.setPower(power);
    }

    /**
     * @return if the Relic Gripper is Closed or not (boolean)
     */
    public boolean isGripClosed(){
        return isClosed;
    }
}
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


    public RelicArm(Robot robot) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;
        this.armMain = hwMap.dcMotor.get("ram");
        this.armMain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.armMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gripper = hwMap.servo.get("rag");
        this.armEx = hwMap.crservo.get("rae");
        this.armEx.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // TODO get positions for gripper
    //TODO get Pos Multiplier for ArmMain and ArmEx (If needed)

    public void openGrip() {
        gripper.setPosition(0);
    }

    public void closeGrip() {
        gripper.setPosition(1);
    }

    public void setArmMainPower(double power) {
        armMain.setPower(power);
    }
    public void setArmExtensionPower(double power) {
        armEx.setPower(power);
    }

    public void initializeRelicArm(){
        armEx.setPower(0);
        armMain.setPower(0);
        openGrip();
    }
}
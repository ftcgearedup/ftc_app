package org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDirectionalDriveTrain;

/**
 * Created by peace on 8/7/2018.
 */

public class HolonomicDriveTrain extends OpMode implements IDirectionalDriveTrain{

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    private boolean driveReversed = false;

    @Override
    public void init() {
        //init motors
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //set powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void pivoting(double pivotSpeed, boolean directionRight) {

        if (directionRight= true) {
            frontLeftPower = -pivotSpeed;
            frontRightPower = pivotSpeed;
            backLeftPower = -pivotSpeed;
            backRightPower = pivotSpeed;
        }
        else {
            frontLeftPower = pivotSpeed;
            frontRightPower = -pivotSpeed;
            backLeftPower = pivotSpeed;
            backRightPower = -pivotSpeed;
        }
    }


    @Override
    public void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    public void driveDirections(double speedfl, double speedfr, double speedbr, double speedbl) {

        frontLeftPower = speedfl;
        frontRightPower = speedfr;
        backLeftPower = speedbr;
        backRightPower = speedbl;
    }


    //unused

    @Override
    public void directionalDrive(double angleDegrees, double speed, int targetDistance, boolean nonBlocking) {

    }

    @Override
    public void drive(double speedX, double speedY) {

    }
    @Override
    public void pivot(double pivotSpeed) {

    }

    @Override
    public void drive(double speedY, int targetDistance) {

    }

    @Override
    public boolean isDriveTrainBusy() {
        return false;
    }

    @Override
    public void loop() {

    }

}
package org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDirectionalDriveTrain;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.RoverRuckusRobot;

/**
 * Created by peace on 9/16/2018.
 */

public class MecanumDrivetrain extends LinearOpMode implements IDirectionalDriveTrain {


    private OpMode opMode;

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor bl;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public MecanumDrivetrain(Robot robot) {
    }

    /**
     * A nested class used to construct {@link MecanumDrivetrain} instances.
     * The builder pattern is utilized here to simplify the construction of rather complex
     * {@link MecanumDrivetrain} objects with differing gearing ratios and motor directions.
     */
    public static class Builder {
        private MecanumDrivetrain mecanumDrivetrain;

        /**
         * Create a new builder object with a reference to the utilizing robot.
         *
         * @param robot the robot that will using the resulting {@link MecanumDrivetrain}
         *              instance returned by calling {@link #build()}.
         */
        public Builder(Robot robot) {
            this.mecanumDrivetrain = new MecanumDrivetrain(robot);
        }

        /**
         * Return the built {@link HDriveTrain} object.
         *
         * @return the final, constructed {@link HDriveTrain} object.
         */
        public MecanumDrivetrain build() {
            // todo: check if all options are set
            return mecanumDrivetrain;
        }
    }

    private MecanumDrivetrain( RoverRuckusRobot roverRuckusRobot){
        this.opMode = roverRuckusRobot.getCurrentOpMode();
        HardwareMap hWMap = opMode.hardwareMap;
        fl = hWMap.dcMotor.get("fl");
        fr = hWMap.dcMotor.get("fr");
        bl = hWMap.dcMotor.get("bl");
        br = hWMap.dcMotor.get("br");

        // set all motors to brake
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">","in run opmode");
        //init motors
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        //set powers
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);

        telemetry.addData("motor speeds","fl "+ frontLeftPower + " fr "+frontRightPower + " bl "+ backLeftPower + " br "+ backRightPower);
        telemetry.update();
    }

    @Override
    public void drive(double speedX, double speedY) {
        frontLeftPower = speedX;
        frontRightPower = speedY;
        backLeftPower = speedY;
        backRightPower = speedX;
        telemetry.addData("motor speeds","fl "+ frontLeftPower + " fr "+frontRightPower + " bl "+ backLeftPower + " br "+ backRightPower);
        telemetry.update();
    }

    @Override
    public void pivot(double pivotSpeed) {
        frontLeftPower = pivotSpeed;
        frontRightPower = pivotSpeed;
        backLeftPower = pivotSpeed;
        backRightPower = pivotSpeed;
       // telemetry.addData("motor speeds","fl "+ frontLeftPower + " fr "+frontRightPower + " bl "+ backLeftPower + " br "+ backRightPower);
       // telemetry.update();
    }

    @Override
    public void drive(double speedY, int targetDistance) {
        frontLeftPower = speedY;
        frontRightPower = -speedY;
        backLeftPower = -speedY;
        backRightPower = speedY;
        telemetry.addData("motor speeds","fl "+ frontLeftPower + " fr "+frontRightPower + " bl "+ backLeftPower + " br "+ backRightPower);
        telemetry.update();
    }

    @Override
    public boolean isDriveTrainBusy() {
        return false;
    }

    @Override
    public void stopDriveMotors() {
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
        telemetry.addData("motor speeds","fl "+ frontLeftPower + " fr "+frontRightPower + " bl "+ backLeftPower + " br "+ backRightPower);
        telemetry.update();
    }

    @Override
    public void directionalDrive(double angleDegrees, double speed, int targetDistance, boolean nonBlocking) {

    }


}
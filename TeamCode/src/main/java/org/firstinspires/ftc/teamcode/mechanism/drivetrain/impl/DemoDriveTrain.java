package org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDirectionalDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDriveTrain;

/**
 * This class implements control of a standard demoing drive train, or simple tank drive which has
 * two omniwheels on the front, as well as two wheels on the back, each driven by their own motor
 */



public class DemoDriveTrain implements IDriveTrain {
    private DcMotor leftDrive, rightDrive;
    private DcMotor.RunMode mode;

    private OpMode opMode;

    private boolean isRunningToPosition;

    private double currentSpeedY;
    private double currentPivot;
    public static class Builder{
        private DemoDriveTrain demoDriveTrain;
        /**
         * Create a new builder object with a reference to the utilizing robot.
         *
         * @param robot the robot that will using the resulting {@link HDriveTrain}
         *              instance returned by calling {@link #build()}.
         */

        //this is how the builder method works, it is a class within a class, which can be
        //considered bad practice (because it's difficult to find), but here, one can do builder
        //things, down in line 70, you may notice that the programmer has the capability to check
        //that every value was set correctly, it's not done yet, but its very good to do

        public Builder(Robot robot) {
            this.demoDriveTrain = new DemoDriveTrain(robot);
        }
        /**
         * Set the direction of the left drive train motor.
         *
         * @see DcMotor.Direction for more documentation on the direction enumeration type
         * @param direction the direction the left motor should drive. Please note that reverse does not
         *                  necessarily correspond to counterclockwise and likewise for clockwise.
         *
         * @return this builder object so that method calls can be chained
         */
        public DemoDriveTrain.Builder setLeftMotorDirection(DcMotor.Direction direction) {
            demoDriveTrain.leftDrive.setDirection(direction);
            return this;
        }
        public DemoDriveTrain.Builder setRightMotorDirection(DcMotor.Direction direction) {
            demoDriveTrain.rightDrive.setDirection(direction);
            return this;
        }

        /**
         * Return the built {@link HDriveTrain} object.
         *
         * @return the final, constructed {@link HDriveTrain} object.
         */
        public DemoDriveTrain build() {
            // todo: check if all options are set
            return demoDriveTrain;
        }


    }
    private DemoDriveTrain(Robot robot) {
        this.opMode = robot.getCurrentOpMode();
        HardwareMap hWMap = opMode.hardwareMap;

        leftDrive = hWMap.dcMotor.get("l");
        rightDrive = hWMap.dcMotor.get("r");


        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set all motors to brake
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }




    @Override
    public boolean isDriveTrainBusy() {
        return (leftDrive.isBusy() && rightDrive.isBusy());
    }


    //the pivot method, its usage is found two classes below, in DemoOp, However it can also be used
    //elsewere, such as if another programmer were to create a 'DemoAutonomous' program, and use a
    //DemoRobot
    @Override
    public void pivot(double pivotSpeed) {
        this.currentPivot = pivotSpeed;

        leftDrive.setPower(currentSpeedY - pivotSpeed);
        rightDrive.setPower(currentSpeedY + pivotSpeed);

    }

    @Override
    public void stopDriveMotors() {
        this.currentSpeedY = 0;
        this.currentPivot = 0;

        leftDrive.setPower(0);
        rightDrive.setPower(0);
 }

    public void setRunMode(DcMotor.RunMode runMode) {
        leftDrive.setMode(runMode);
        rightDrive.setMode(runMode);

        this.mode = runMode;
    }

    //drive method, 'targetDistance' is not needed
    @Override
    public void drive(double speedY, int targetDistance) {
        this.currentSpeedY = speedY;

        leftDrive.setPower(this.currentPivot + speedY);
        rightDrive.setPower(-this.currentPivot + speedY);
    }



}

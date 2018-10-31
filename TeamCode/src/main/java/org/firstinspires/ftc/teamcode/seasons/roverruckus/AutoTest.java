package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.Direction;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.EncoderValues;

@Autonomous(name = "MechenumAuto", group = "Autonomous")
public class AutoTest extends OpMode {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;

    @Override
    public void init() {
        // init the Wheels
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");


        // set wheel direction
        this.setDirection();
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDriveMode(DcMotor.RunMode modee) {
        frontLeft.setMode(modee);
        frontRight.setMode(modee);
        backLeft.setMode(modee);
        backRight.setMode(modee);
    }

    //for backwards use negative power
    public void forward(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDistance = frontRight.getCurrentPosition();
        while (currentDistance < targetDistance) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }

        stop();
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void sideLeft(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = frontRight.getCurrentPosition();
        while (currentDistance < targetDistance) {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

    }

    public void sideRight(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = frontLeft.getCurrentPosition();
        while (currentDistance < targetDistance) {
            currentDistance = 0;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }

    }
    //clockwise is 0 cc is 1

    public void pivot(double degree, double power, Direction direction) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (direction == Direction.clockwise) {
            double currentDegree = frontLeft.getCurrentPosition();
            while (currentDegree < degree) {
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(power);
                backRight.setPower(-power);
            }
            stop();
        } else if (direction == Direction.counterclockwise) {
            double currentDegree = frontRight.getCurrentPosition();
            while (currentDegree < degree) {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
            stop();
        }
    }


    @Override
    public void loop() {
        pivot(1, 1, Direction.clockwise);
        forward(743, 1);
        sideRight(657, 1);
        sideLeft(948, 1);
    }
}

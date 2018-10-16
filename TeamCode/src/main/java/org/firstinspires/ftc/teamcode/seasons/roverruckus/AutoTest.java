package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mechenum Test", group = "TeleOp")
public class AutoTest extends OpMode{
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
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    //for backwards use negative power
public void forward(double distance, double power) {
    frontLeft.setPower(power);
    frontRight.setPower(power);
    backLeft.setPower(power);
    backRight.setPower(power);
}
public void sideLeft(double distance, double power) {
    frontLeft.setPower(-power);
    frontRight.setPower(power);
    backLeft.setPower(-power);
    backRight.setPower(power);
}
    public void sideRight(double distance, double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    @Override
    public void loop() {

    }
}

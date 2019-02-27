package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MechenumTeleOp extends OpMode {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor intakeLift;
    private DcMotor lift;
    private Servo lBucket;
    private DcMotor hook;
    @Override
    public void init() {
        // init the motors
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");
        intake = hardwareMap.dcMotor.get("intake");
        intakeLift = hardwareMap.dcMotor.get("intakeLift");
        lift = hardwareMap.dcMotor.get("lift");
        lBucket = hardwareMap.servo.get("lbucket");
        hook = hardwareMap.dcMotor.get("hook");


        // set wheel direction
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        //init attachments
        intake.setPower(0);
        lift.setPower(0);
        lBucket.setPosition(.5);
        hook.setPower(0);

        // set deadzone
        gamepad1.setJoystickDeadzone(0.1f);
    }

    public void move() {
        //the right Joystick controls movement, and the left controls turning.
        //in order to counteract the differences between the basic formula for strafing and the one for moving forwards and backwards,
        //we use trigonometry to derive relevant powers for each wheel tio move at the correct angles.
        // translating polar coordanates of joyst   ick to polar coordinates of mechenum drive
        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double leftX = gamepad1.left_stick_x;
        final double fr = r * Math.cos(robotAngle) + leftX;
        final double fl = r * Math.sin(robotAngle) - leftX;
        final double bl = r * Math.sin(robotAngle) + leftX;
        final double br = r * Math.cos(robotAngle) - leftX;

        double intakePower=0.0;

        if (gamepad1.left_trigger > 0) {
            frontRight.setPower(fr / 2);
            frontLeft.setPower(fl / 2);
            backLeft.setPower(bl / 2);
            backRight.setPower(br / 2);
        } else if (gamepad1.right_trigger > 0) {
            frontRight.setPower(fr / 4);
            frontLeft.setPower(fl / 4);
            backLeft.setPower(bl / 4);
            backRight.setPower(br / 4);
        } else {
            frontRight.setPower(fr);
            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            backRight.setPower(br);
        }
        //attachments

       if ( gamepad2.right_trigger == 1){
            lBucket.setPosition(1);
       } else if (gamepad2.left_trigger == 1){
            lBucket.setPosition(0);
       }

       intakePower = gamepad2.left_stick_y;
        intake.setPower(intakePower);

        double liftPower;
        liftPower = gamepad2.right_stick_y;
        lift.setPower(liftPower);

     if (gamepad2.dpad_up){
         intakeLift.setPower(.7);
     } else if (gamepad2.dpad_down) {
         intakeLift.setPower(-.7);
     }else {
         intakeLift.setPower(0);
     }

     if(gamepad1.dpad_up){
         hook.setPower(1);
     }else if (gamepad1.dpad_down) {
         hook.setPower(-1);
     }else {
         hook.setPower(0);
     }
        telemetry.addData("motor speeds","fl "+ fl + " fr "+fr + " bl "+ bl + " br "+ br);
        telemetry.addData("enc values",
                "fl " + frontLeft.getCurrentPosition() +
                        " fr " + frontRight.getCurrentPosition() +
                        " bl " + backLeft.getCurrentPosition() +
                        " br " + backRight.getCurrentPosition());

        //telemetry.addData("intake ", "intake ",intakePower);
        telemetry.update();
    }
    @Override
    public void loop() {
        move();
    }
}

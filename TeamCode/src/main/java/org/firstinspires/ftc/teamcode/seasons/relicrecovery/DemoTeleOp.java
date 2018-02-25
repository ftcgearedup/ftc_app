package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *
 * Created by daniel on 11/12/17.
 */

@TeleOp(name = "Promote Video TeleOp", group = "Promote")
public class DemoTeleOp extends LinearOpMode {

    private DcMotor right;
    private DcMotor left;
//    private DcMotor head;
    private Servo gripper;
    @Override
    public void runOpMode() throws InterruptedException {
        //set deadzones
        gamepad1.setJoystickDeadzone(.02f);

        //initiate motors

        left = hardwareMap.dcMotor.get("l");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right = hardwareMap.dcMotor.get("r");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        head = hardwareMap.dcMotor.get("h");
//        head.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper = hardwareMap.servo.get("gripper");
        gripper.setPosition(.5);
        waitForStart();
        double speedLeft;
        double speedRight;
//        double speedHead;


        while (opModeIsActive()) {
            speedLeft = gamepad1.left_stick_y;
            speedRight = -gamepad1.right_stick_y;

            left.setPower(speedLeft);
            right.setPower(speedRight);
            if(gamepad1.a){
                gripper.setPosition(.2); //closed
            } else {
                gripper.setPosition(.5); //open
            }
//            if (gamepad1.right_bumper) {
//                speedHead = 0.3;
//                head.setPower(speedHead);
//            } else if (gamepad1.left_bumper) {
//                speedHead = -0.3;
//                head.setPower(speedHead);
//            } else{
//                head.setPower(0.0);
//            }
        }
    }
}


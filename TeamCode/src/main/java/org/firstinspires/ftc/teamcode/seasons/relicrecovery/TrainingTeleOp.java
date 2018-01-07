package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 *
 * Created by daniel on 11/12/17.
 */

@TeleOp(name = "Training TeleOp", group = "Promote Video")
public class TrainingTeleOp extends LinearOpMode {

    private DcMotor right;
    private DcMotor left;
    private DcMotor head;
    @Override
    public void runOpMode() throws InterruptedException {
        //set deadzones
        gamepad1.setJoystickDeadzone(.02f);

        //initiate motors

      left = hardwareMap.dcMotor.get("l");
      right = hardwareMap.dcMotor.get("r");
      head = hardwareMap.dcMotor.get("h");
        waitForStart();
        double speedLeft;
        double speedRight;
        double speedHead;

        while (opModeIsActive()) {
            speedLeft = gamepad1.left_stick_x;
            speedRight = gamepad1.right_stick_x;
            speedHead = 0;

            if(gamepad1.left_bumper){
                speedHead = -1;
            } else if(gamepad1.right_bumper){
                speedHead = 1;
            } else {
                speedHead = 0;
            }

            left.setPower(speedLeft);
            right.setPower(speedRight);
            head.setPower(speedHead);

        }

    }
}

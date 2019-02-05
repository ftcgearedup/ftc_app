package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 *
 * Created by daniel on 11/12/17.
 */

@TeleOp(name = "OtisDemoOp", group = "Demo")
public class OtisDemoOp extends LinearOpMode {

    private OtisDemoRobot robot;

    private boolean speedInhibitorBoolean = false;


    private DcMotor right;
    private DcMotor left;
    private DcMotor middle;

    private JSONConfigOptions configOptions;

    private float joystickDeadzone;

    //    private DcMotor head;
    private Servo gripper;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new OtisDemoRobot(this);
        //initiate motors

        left = hardwareMap.dcMotor.get("l");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right = hardwareMap.dcMotor.get("r");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middle = hardwareMap.dcMotor.get("m");
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        head = hardwareMap.dcMotor.get("h");
//        head.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        gripper = hardwareMap.servo.get("gripper");
//        gripper.setPosition(.5);
        waitForStart();
        double speedX;
        double speedY;
        double pivot;

        boolean passcode;
        boolean nothingElseIsPressed;

//        double speedHead;
//        double gripperPosition;


        while (opModeIsActive()) {
            passcode = gamepad1.left_trigger > .7 && gamepad1.right_trigger > .7 && (gamepad1.y || gamepad1.b) && gamepad1.dpad_up;
            nothingElseIsPressed = gamepad1.a == false &&
                    gamepad1.x==false && gamepad1.dpad_down==false &&
                    gamepad1.dpad_left == false && gamepad1.dpad_right == false &&
                    gamepad1.back == false && gamepad1.start == false &&
                    gamepad1.left_stick_y <.2 && gamepad1.right_stick_y <.2 &&
                    gamepad1.left_stick_button == false && gamepad1.right_stick_button == false &&
                    gamepad1.left_stick_x <.2 && gamepad1.right_stick_x <.2;
            //standard tank drive robots
//            speedLeft = gamepad1.left_stick_y;
//            speedRight = -gamepad1.right_stick_y;

//otis
            speedX = gamepad1.right_stick_x;
            speedY = -gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;



            if(passcode && gamepad1.y == true && nothingElseIsPressed && speedInhibitorBoolean == false)
            {
                speedInhibitorBoolean = true;

            } else if (passcode && gamepad1.b == true && nothingElseIsPressed &&speedInhibitorBoolean ==true)
            {
                speedInhibitorBoolean = false;
            }

            telemetry.addData("SpeedInhibited? ", speedInhibitorBoolean);
            telemetry.update();
            

            if(speedInhibitorBoolean)
            {
                speedX = Range.clip(speedX, -.1,.1);
                speedY = Range.clip(speedY, -.1,.1);
                pivot = Range.clip(pivot, -.1,.1);
            }

            robot.getHDriveTrain().pivot(pivot);
            robot.getHDriveTrain().drive(speedX, speedY);



            telemetry.addData("speedX",speedX);
            telemetry.addData("speedY",speedY);
            telemetry.addData("pivot",pivot);

        }
    }
}



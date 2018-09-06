package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HolonomicDriveTrain;

/**
 * Created by peace on 8/28/2018.
 */
@Disabled
@TeleOp(name = "HolonomicTeleOp", group = "tele-op")
public class HolonomicTeleOp extends LinearOpMode {

    private HolonomicDriveTrain robot;

    public void runOpMode() {

        robot = new HolonomicDriveTrain();
        //set deadzones
        gamepad1.setJoystickDeadzone(.02f);

        waitForStart();

        init();

        while (opModeIsActive()) {

            //this section sets the joystick controls for basic movment.

            //move backwards
            if (gamepad1.right_stick_y < 0) {
                robot.driveDirections(-50, -50, 50, 50);
            }
            //move forwards
            if (gamepad1.right_stick_y > 0) {
                robot.driveDirections(50, 50, -50, -50);
            }
            //move right
            if (gamepad1.right_stick_x > 0) {
                robot.driveDirections(-50, 50, 50, -50);
            }
            //move left
            if (gamepad1.right_stick_y < 0) {
                robot.driveDirections(50, -50, -50, 50);
            }

            // this section controls pivoting.

            //turn right
            if (gamepad1.left_stick_x > 0) {
                robot.pivoting(50, true);
            }
            // turn left
            if (gamepad1.left_stick_x > 0) {
                robot.pivoting(50, true);
            }

        }
    }

}

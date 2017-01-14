package org.firstinspires.ftc.teamcode.seasons.velocityvortex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ftc6347 on 10/16/16.
 */
@TeleOp(name = "Teleop", group = "teleop")
public class Teleop extends LinearOpMode {

    private static final float JOYSTICK_DEADZONE = 0.2f;

    private ZoidbergHardware robot;

    private float frontLeftPower;
    private float frontRightPower;
    private float backLeftPower;
    private float backRightPower;

    private boolean yButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ZoidbergHardware(hardwareMap);

        // run without encoders
        robot.getBackLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getBackRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getFrontRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad1.setJoystickDeadzone(JOYSTICK_DEADZONE);
        gamepad2.setJoystickDeadzone(JOYSTICK_DEADZONE);

        waitForStart();

        while(opModeIsActive()) {

            // reset variables
            frontLeftPower = 0;
            frontRightPower = 0;
            backLeftPower = 0;
            backRightPower = 0;

            handleStrafe();
            handlePivot();

            // right trigger divides the speed of every drive motor by four
            if(gamepad1.right_trigger > 0) {
                frontLeftPower /= 4;
                frontRightPower /= 4;
                backLeftPower /= 4;
                backRightPower /= 4;
            }

            // set the actual motor powers
            robot.getFrontLeftDrive().setPower(frontLeftPower);
            robot.getFrontRightDrive().setPower(frontRightPower);
            robot.getBackLeftDrive().setPower(backLeftPower);
            robot.getBackRightDrive().setPower(backRightPower);

            handleIntake();
            handleLauncher();

            handleTelemetry();

            idle();
        }
    }

    private void handleIntake() {
        if (gamepad2.b){
            robot.getDoor3().setPosition(0.25);
            robot.getIntakeMotor().setPower(-1.0);
        }
        else if(gamepad2.right_stick_y >= 0.2 || gamepad2.right_stick_y <= -0.2 ){
            robot.getDoor3().setPosition(0.55);
            robot.getIntakeMotor().setPower(gamepad2.right_stick_y);
        }
        else {
            robot.getIntakeMotor().setPower(0);
            robot.getDoor3().setPosition(0.25);
        }
    }

    private void handleLauncher() throws InterruptedException {

        // if the Y button on the second gamepad was previously pressed
        if(yButtonPressed) {
            // if white is essentially detected by the ODS sensor
            if (robot.getDiskOds().getRawLightDetected() > 1) {
                // run the launcher motor at a slower speed to find the black stripe
                robot.getLauncherMotor().setPower(0.3);
            } else {
                // stop the motor once the black stripe is detected
                robot.getLauncherMotor().setPower(0);

                // forget that the Y button was pressed so that the launcher
                // motor will not run until it detects the black stripe
                yButtonPressed = false;
            }
        // run the launcher motor at full speed to launch the particle when the Y button is pressed
        } else if(gamepad2.y) {
            robot.getRuntime().reset();
            
            while(robot.getRuntime().milliseconds() < 900) {
                robot.getLauncherMotor().setPower(1);
            }
            
            robot.getLauncherMotor().setPower(0);

            // remember that the Y button was pressed so that the code that runs the launcher motor
            // until it detects the black stripe in the above if-statement will be executed
            yButtonPressed = true;
        }
    }

    private void handleTelemetry() {

        telemetry.addData("ods2 : ", robot.getDiskOds().getRawLightDetected());
        telemetry.addData("ods3", robot.getOds3().getRawLightDetected());
        telemetry.addData("launcher ods", robot.getLauncherOds().getLightDetected());
        telemetry.addData("front range", robot.getFrontRange().cmUltrasonic());

        telemetry.update();
    }

    private void handlePivot() {
        frontLeftPower -= gamepad1.left_stick_x;
        frontRightPower -= gamepad1.left_stick_x;
        backLeftPower -= gamepad1.left_stick_x;
        backRightPower -= gamepad1.left_stick_x;
    }

    private void handleStrafe() {
        frontLeftPower += -gamepad1.right_stick_y + gamepad1.right_stick_x;
        frontRightPower += gamepad1.right_stick_y + gamepad1.right_stick_x;
        backLeftPower += -gamepad1.right_stick_y - gamepad1.right_stick_x;
        backRightPower += gamepad1.right_stick_y - gamepad1.right_stick_x;
    }
}
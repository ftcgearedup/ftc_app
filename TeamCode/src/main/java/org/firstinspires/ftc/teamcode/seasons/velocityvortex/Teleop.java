package org.firstinspires.ftc.teamcode.seasons.velocityvortex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ftc6347 on 10/16/16.
 */
@TeleOp(name = "Teleop", group = "teleop")
public class Teleop extends LinearOpModeBase {

    private static final float JOYSTICK_DEADZONE = 0.2f;

    private float frontLeftPower;
    private float frontRightPower;
    private float backLeftPower;
    private float backRightPower;

    private boolean yButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // run without encoders
        getBackLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getBackRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFrontRightDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            getFrontLeftDrive().setPower(frontLeftPower);
            getFrontRightDrive().setPower(frontRightPower);
            getBackLeftDrive().setPower(backLeftPower);
            getBackRightDrive().setPower(backRightPower);

            handleIntake();
            handleLauncher();
            handleCapBallMechanism();

            handleTelemetry();

            idle();
        }
    }

    private void handleCapBallMechanism() {
        double spoolMotorSpeed = -gamepad2.left_stick_y;
        getSpoolMotor1().setPower(spoolMotorSpeed);
        getSpoolMotor2().setPower(spoolMotorSpeed);
    }

    private void handleIntake() {
        if (gamepad2.b){
            getDoor3().setPosition(0.25);
            getIntakeMotor().setPower(-1.0);
        }
        else if(gamepad2.right_stick_y >= 0.2 || gamepad2.right_stick_y <= -0.2 ){
            getDoor3().setPosition(0.55);
            getIntakeMotor().setPower(gamepad2.right_stick_y);
        }
        else {
            getIntakeMotor().setPower(0);
            getDoor3().setPosition(0.25);
        }
    }

    private void handleLauncher() throws InterruptedException {

        // if the Y button on the second gamepad was previously pressed
        if(yButtonPressed) {
            // if white is essentially detected by the ODS sensor
            if (getDiskOds().getRawLightDetected() > 1) {
                // run the launcher motor at a slower speed to find the black stripe
                getLauncherMotor().setPower(0.3);
            } else {
                // stop the motor once the black stripe is detected
                getLauncherMotor().setPower(0);

                // forget that the Y button was pressed so that the launcher
                // motor will not run until it detects the black stripe
                yButtonPressed = false;
            }
        // run the launcher motor at full speed to launch the particle when the Y button is pressed
        } else if(gamepad2.y) {
            getRobotRuntime().reset();
            
            while(getRobotRuntime().milliseconds() < 900) {
                getLauncherMotor().setPower(1);
            }
            
            getLauncherMotor().setPower(0);

            // remember that the Y button was pressed so that the code that runs the launcher motor
            // until it detects the black stripe in the above if-statement will be executed
            yButtonPressed = true;
        }
    }

    private void handleTelemetry() {

        telemetry.addData("ods2 : ", getDiskOds().getRawLightDetected());
        telemetry.addData("ods3", getOds3().getRawLightDetected());
        telemetry.addData("launcher ods", getLauncherOds().getLightDetected());
        telemetry.addData("front range", getFrontRange().cmUltrasonic());

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

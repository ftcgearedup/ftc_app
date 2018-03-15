package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * This class is the Relic Recovery competition robot tele-op program.
 *
 * CONTROLS USED
 *
 * DRIVER 1:
 *
 * Right Stick (X, Y) - Movement
 * Left Stick (X)     - Rotation
 * Right Trigger      - Slow Driving
 *
 * Y Button           - Jewel Knocker Lower and Center Rotator
 *
 * DRIVER 2:
 *
 * Left Stick (Y)     - Glyph Intake Height
 * Right Stick (Y)    - Glyph Intake Run (Up for Inward, Down for Outward)
 * A Button           - Relic Arm Grip Toggle
 * Up Button          - Relic Arm Rotator Up
 * Down Button        - Relic Arm Rotator Down
 * Left Trigger       - Relic Arm Retract
 * Right Trigger      - Relic Arm Extend
 */
@TeleOp(name = "TELEOP", group = "teleop")
public class RobotTeleOp extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private JSONConfigOptions configOptions;

    private float joystickDeadzone;
    private int glyphColorThreshold;
    private int jewelColorSensorLEDFlash;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);
        this.configOptions = robot.getOptionsMap();
        joystickDeadzone = (float) configOptions.retrieveAsDouble("teleopJoystickDeadzone");
        glyphColorThreshold = configOptions.retrieveAsInt("gcsThreshold");
        jewelColorSensorLEDFlash = configOptions.retrieveAsInt("jcsFlashMS");

        gamepad1.setJoystickDeadzone(joystickDeadzone);
        gamepad2.setJoystickDeadzone(joystickDeadzone);

        robot.getJewelKnocker().retractArm();

        waitForStart();

        double speedX;
        double speedY;
        double pivot;

        double liftMotorPower;

        boolean relicTogglePressed = true;
        boolean relicGripperOpen = false;
        boolean isJewelColorSensorLEDLit = false;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            speedX = gamepad1.right_stick_x;
            speedY = -gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;

            liftMotorPower = -gamepad2.left_stick_y;

            // slow down robot with right trigger                   SLOW DRIVING
            if(gamepad1.right_trigger > 0) {
                speedY /= 3;
                pivot /= 3;
                speedX /= 2;
            }

            // Relic arm motor control                              RELIC ARM CONTROLS
            if(gamepad2.right_trigger > 0) {
                robot.getRelicArm().setArmMainPower(gamepad2.right_trigger);
            } else {
                robot.getRelicArm().setArmMainPower(-gamepad2.left_trigger);
            }

            // Relic rotation servo control
            if(gamepad2.dpad_up) {
                robot.getRelicArm().raiseArmRotation();
            } else if(gamepad2.dpad_down) {
                robot.getRelicArm().lowerArmRotation();
            }

            // Relic gripper toggle
            if(gamepad2.a && relicTogglePressed) {
                if(relicGripperOpen) {
                    robot.getRelicArm().closeGripper();
                } else {
                    robot.getRelicArm().openGripper();
                }

                relicGripperOpen = !relicGripperOpen;
            }

            relicTogglePressed = !gamepad2.a;

            // Jewel arm control                                    JEWEL CONTROLS
            if (gamepad1.y){
                robot.getJewelKnocker().extendArm();
                robot.getJewelKnocker().centerRotation();
            } else {
                robot.getJewelKnocker().retractArm();
                robot.getJewelKnocker().leftRotation();
            }

            if(robot.getGlyphLift().getColorSensor().red() > glyphColorThreshold) {
                if(timer.milliseconds() > jewelColorSensorLEDFlash) {
                    timer.reset();

                    isJewelColorSensorLEDLit = !isJewelColorSensorLEDLit;

                    if(isJewelColorSensorLEDLit) {
                        robot.getJewelKnocker().enableLED();
                    } else {
                        robot.getJewelKnocker().disableLED();
                    }
                }
            }

            // glyph lift intake power control
            robot.getGlyphLift().setGlyphIntakeMotorPower(gamepad2.right_stick_y);

            telemetry.addData("Red Level", robot.getJewelKnocker().getRed());
            telemetry.addData("Blue Level", robot.getJewelKnocker().getBlue());

            telemetry.addData("GCS Red Level", robot.getGlyphLift().getColorSensor().red());
            telemetry.addData("GCS Blue Level", robot.getGlyphLift().getColorSensor().blue());

            telemetry.addData("Glyph Touch Sensor", robot.getGlyphLift().getGlyphTouchSensor().isPressed());
            telemetry.addData("Glyph Lift Touch Sensor", !robot.getGlyphLift().getLiftTouchSensor().getState());

            telemetry.addData("left lift motor position", robot.getGlyphLift().getLiftLeftMotorPosition());
            telemetry.addData("right lift motor position", robot.getGlyphLift().getLiftRightMotorPosition());

            telemetry.addData("relic arm motor position", robot.getRelicArm().getArmMotorPosition());

            telemetry.update();

            robot.getHDriveTrain().pivot(pivot);
            robot.getHDriveTrain().drive(speedX, speedY);

            robot.getGlyphLift().setLiftMotorPower(liftMotorPower);
        }
    }
}

package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
@TeleOp(name = "TELEOP \uD83C\uDFAE", group = "teleop")
@Disabled
public class RobotTeleOp extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private JSONConfigOptions configOptions;

    private float joystickDeadzone;
    private int glyphColorThreshold;
    private int jewelColorSensorLEDFlash;
    private int intakeHeightValue;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);
        this.configOptions = robot.getOptionsMap();
        joystickDeadzone = (float) configOptions.retrieveAsDouble("teleopJoystickDeadzone");
        glyphColorThreshold = configOptions.retrieveAsInt("gcsThreshold");
        jewelColorSensorLEDFlash = configOptions.retrieveAsInt("jcsFlashMS");
        intakeHeightValue = configOptions.retrieveAsInt("teleopIntakeHeightValue");

        gamepad1.setJoystickDeadzone(joystickDeadzone);
        gamepad2.setJoystickDeadzone(joystickDeadzone);

        robot.getJewelKnocker().retractArm();
        robot.getGlyphLift().setIntakeInitializePosition();

        waitForStart();

        double speedX;
        double speedY;
        double pivot;

        double liftMotorPower;

        boolean relicTogglePressed = true;
        boolean relicGripperOpen = false;
        boolean isJewelColorSensorLEDLit = true;
        ElapsedTime timer = new ElapsedTime();

        robot.getGlyphLift().setIntakeHalfOpenPosition();

        // initialize vuforia
        //robot.getVisionHelper().initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);

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
            if(gamepad2.dpad_up || gamepad2.right_bumper) {
                robot.getRelicArm().raiseArmRotation();
            } else if(gamepad2.dpad_down || gamepad2.left_bumper) {
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

//                                                              GLYPH LIFT AND INTAKE CONTROLS

            // glyph lift intake power control
            robot.getGlyphLift().setGlyphIntakeMotorPower(gamepad2.right_stick_y);

            /*
             * This adjusts the position of the Glyph Lift Intake. If you are running the intake,
             * the intake sets to the Grip position to grab the cubes. If you aren't pulling
             * in glyphs, the intake will go to a 45° angle if it's near the wheel cover,
             * or fully open if it's not near the cover.
             */
            if(gamepad2.right_stick_y < 0) {
                telemetry.addData("intake", "grip position");

                robot.getGlyphLift().setIntakeGripPosition();
            } else if(robot.getGlyphLift().getLiftLeftMotorPosition() >= intakeHeightValue) {
                telemetry.addData("intake", "fully open position");

                robot.getGlyphLift().setIntakeFullyOpenPosition();
            } else {
                telemetry.addData("intake", "half open position");

                robot.getGlyphLift().setIntakeHalfOpenPosition();
            }

//            // toggle camera LED flash
//            if(robot.getGlyphLift().getColorSensor().red() > glyphColorThreshold) {
//                CameraDevice.getInstance().setFlashTorchMode(isJewelColorSensorLEDLit);
//
//                if(timer.milliseconds() > jewelColorSensorLEDFlash) {
//                    timer.reset();
//
//                    isJewelColorSensorLEDLit = !isJewelColorSensorLEDLit;
//                }
//            } else {
//                CameraDevice.getInstance().setFlashTorchMode(false);
//                isJewelColorSensorLEDLit = true;
//            }


            telemetry.addData("left intake servo", robot.getGlyphLift().intakeArmLeft.getPosition());
            telemetry.addData("right intake servo", robot.getGlyphLift().intakeArmRight.getPosition());

            telemetry.addData("Red Level", robot.getJewelKnocker().getRed());
            telemetry.addData("Blue Level", robot.getJewelKnocker().getBlue());

            telemetry.addData("GCS Red Level", robot.getGlyphLift().getColorSensor().red());
            telemetry.addData("GCS Blue Level", robot.getGlyphLift().getColorSensor().blue());

            telemetry.addData("Glyph Touch Sensor", robot.getGlyphLift().getGlyphTouchSensor().isPressed());
            telemetry.addData("Glyph Lift Touch Sensor", !robot.getGlyphLift().getLiftTouchSensor().getState());

            telemetry.addData("left lift motor position", robot.getGlyphLift().getLiftLeftMotorPosition());
            telemetry.addData("right lift motor position", robot.getGlyphLift().getLiftRightMotorPosition());

            telemetry.addData("left intake encoder position", robot.getGlyphLift().getIntakeLeftMotorPosition());
            telemetry.addData("right intake encoder position", robot.getGlyphLift().getIntakeRightMotorPosition());

            telemetry.addData("relic arm motor position", robot.getRelicArm().getArmMotorPosition());

            telemetry.addData("left range sensor", robot.getLeftRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("right range sensor", robot.getRightRangeSensor().getDistance(DistanceUnit.INCH));

            telemetry.update();

            robot.getHDriveTrain().pivot(pivot);
            robot.getHDriveTrain().drive(speedX, speedY);

            robot.getGlyphLift().setLiftMotorPower(liftMotorPower);
        }
    }
}

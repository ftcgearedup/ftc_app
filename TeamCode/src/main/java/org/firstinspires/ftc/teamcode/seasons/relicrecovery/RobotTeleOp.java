package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 This class is the competition robot tele-op program.

 CONTROLS USED

 DRIVER 1:

 Right Stick   - Movement
 Left Stick (X)- Rotation
 Right Trigger - Slow Driving

 Y             - Jewel Arm Lower and Center Rotator (Hold)

 DRIVER 2:

 Left Stick (Y)      - Glyph Intake Height
 Right Stick (Y)     - Glyph Intake Run (Up for Inward, Down for Outward)

 */
@TeleOp(name = "TELEOP", group = "teleop")
public class RobotTeleOp extends LinearOpMode {
    private RelicRecoveryRobot robot;

    private static final float JOYSTICK_DEADZONE = 0.2f;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);

        gamepad1.setJoystickDeadzone(JOYSTICK_DEADZONE);
        gamepad2.setJoystickDeadzone(JOYSTICK_DEADZONE);

        robot.getJewelKnocker().retractArm();

        waitForStart();

        double speedX;
        double speedY;
        double pivot;

        double liftMotorPower;

        boolean relicTogglePressed = true;
        boolean relicGripperOpen = false;

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

            // glyph lift intake power control
            robot.getGlyphLift().setGlyphIntakeMotorPower(gamepad2.right_stick_y);

            telemetry.addData("Red Level", robot.getJewelKnocker().getRed());
            telemetry.addData("Blue Level", robot.getJewelKnocker().getBlue());

            telemetry.addData("GCS Red Level", robot.getGlyphLift().getColorSensor().red());
            telemetry.addData("GCS Blue Level", robot.getGlyphLift().getColorSensor().blue());

            telemetry.addData("Glyph Touch Sensor", robot.getGlyphLift().getTouchSensor().isPressed());

            telemetry.update();

            robot.getHDriveTrain().pivot(pivot);
            robot.getHDriveTrain().drive(speedX, speedY);

            robot.getGlyphLift().setLiftMotorPower(liftMotorPower);
        }
    }
}

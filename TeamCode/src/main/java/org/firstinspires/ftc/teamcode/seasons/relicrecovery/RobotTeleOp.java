package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;

/**
 * This class is the competition robot tele-op program.
 *
 * CONTROLS USED

 CONTROLLER 1:

 Right Stick   - Movement
 Left Stick (X)- Rotation
 Right Trigger - Slow Driving

 Left Bumper   - Intake Raise or Lower
 Left Trigger  - Intake Run Inward
 Right Bumper  - Intake Run Outward

 Y             - Jewel Arm Extend/Retract (Hold)

 CONTROLLER 2:

 Left Stick (Y)      - Glyph Lift Height
 Left Stick (X)      - Glyph Lift Rotation
 Right Trigger       - Glyph Lift Blue Gripper
 Left_Trigger        - Glyph Lift Red Gripper
 D-Pad Up and Down   - Glyph Lift Align
 Y                   - Glyph Lift Recalibrate

 Right Stick (Press) - Relic Arm Control Mode Switch
 Right Stick (X)     - Relic Arm Move (Main or Extension, depends on mode)
 A                   - Relic Arm Open Grip
 B                   - Relic Arm Close Grip

 UNUSED

 CONTROLLER 1:

 D-Pad
 Left Stick Press and Y
 Right Stick Press
 A
 B
 X

 CONTROLLER 2:
 Left Bumper
 Right Bumper
 D-Pad Left and Right
 Left Stick Press
 Right Stick Up and Down
 X
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

        robot.getGlyphLift().initializeGrippers();
        robot.getIntake().raiseIntake();

        robot.getGlyphLift().initializeGrippers();
        robot.getIntake().raiseIntake();
        robot.getJewelKnocker().retractArm();
        robot.getRelicArm().initializeRelicArm();

        waitForStart();

        double speedX;
        double speedY;
        double pivot;

        ElapsedTime intakeToggleTimer = new ElapsedTime();
        ElapsedTime relicToggleTimer = new ElapsedTime();

        boolean isRaised = true;

        double liftMotorPower;
        double liftRotationMotorPower = 0;

        while (opModeIsActive()) {
            speedX = -gamepad1.right_stick_x;
            speedY = gamepad1.right_stick_y;
            pivot = -gamepad1.left_stick_x;

            liftMotorPower = -gamepad2.left_stick_y;

            // slow down robot with right trigger                   SLOW DRIVING
            if(gamepad1.right_trigger > 0) {
                speedY /= 3;
                pivot /= 3;
                speedX /= 2;
            }

            // Jewel arm control                                    JEWEL CONTROLS
            if (gamepad1.y){
                robot.getJewelKnocker().extendArm();
            } else {
                robot.getJewelKnocker().retractArm();
            }


            // intake raise/lower control                            INTAKE CONTROLS
            if(gamepad1.right_bumper && intakeToggleTimer.milliseconds() > 200) {  //toggle if raised or lowered
                if(isRaised){
                    robot.getIntake().lowerIntake();
                    isRaised = false;

                } else if(!isRaised){
                    robot.getIntake().raiseIntake();
                    isRaised = true;
                }
                intakeToggleTimer.reset();
            }

            // run intake in
            if(gamepad1.left_trigger > 0.1) {
                robot.getIntake().setIntakePower(-1);
                robot.getIntake().closeLinkage();
            } else if(gamepad1.left_bumper){  // run intake in reverse
                robot.getIntake().setIntakePower(1);
                robot.getIntake().closeLinkage();
            } else {
                robot.getIntake().setIntakePower(0);
                robot.getIntake().openLinkage();
            }

            // close/open blue gripper                                 GLYPH CONTROLS
            if(gamepad2.right_trigger > 0.1) {
                robot.getGlyphLift().openBlueGripper();
            } else {
                robot.getGlyphLift().closeBlueGripper();
            }
            // close/open red gripper
            if(gamepad2.left_trigger > 0.1) {
                robot.getGlyphLift().openRedGripper();
            } else {
                robot.getGlyphLift().closeRedGripper();
            }

            // automatic lift rotation motor control
            if(gamepad2.right_bumper) {
                robot.getGlyphLift().setRotationMotorPosition(GlyphLift.RotationMotorPosition.UP);
            } else if(gamepad2.left_bumper) {
                robot.getGlyphLift().setRotationMotorPosition(GlyphLift.RotationMotorPosition.DOWN);
            } else if(gamepad2.dpad_left){
                robot.getGlyphLift().setRotationMotorPower(0.5);
            } else if(gamepad2.dpad_right){
                robot.getGlyphLift().setRotationMotorPower(-0.5);
            } else {
                robot.getGlyphLift().setRotationMotorPower(0);
            }

//            // TRUE is Main Arm Control, False is Extension Control     RELIC ARM CONTROLS
//
//            // mode switcher
//            if(gamepad2.right_stick_button && relicToggleTimer.milliseconds() > 200){
//                mode = !mode;
//                if(mode){
//                    telemetry.addData("Right Stick Mode", "Main Arm");
//                } else {
//                    telemetry.addData("Right Stick Mode", "Extension");
//                }
//                relicToggleTimer.reset();
//                telemetry.update();
//            }
            // control Extension
          //  robot.getRelicArm().setArmExtensionPower(Math.copySign(0.3, gamepad2.right_stick_x));

            if(gamepad2.b) {
                robot.getRelicArm().setArmExtensionPower(0.3);
            } else if (gamepad2.x) {
                robot.getRelicArm().setArmExtensionPower(-0.3);
            } else {
                robot.getRelicArm().setArmExtensionPower(0);
            }
            // control Main Arm
            robot.getRelicArm().setArmMainPower(Range.clip(gamepad2.right_stick_y, -0.2, 0.2));

            // relic gripper controls
            if(gamepad2.a && relicToggleTimer.milliseconds() > 200) {
                if (robot.getRelicArm().isGripClosed()) {
                    robot.getRelicArm().openGrip();
                    relicToggleTimer.reset();
                } else {
                    robot.getRelicArm().closeGrip();
                    relicToggleTimer.reset();
                }
            }

            if(gamepad2.y) {
                robot.getGlyphLift().rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("Red Level", robot.getJewelKnocker().getRed());
            telemetry.addData("Blue Level", robot.getJewelKnocker().getBlue());
            telemetry.update();

            robot.getHDriveTrain().pivot(pivot);
            robot.getHDriveTrain().drive(speedX, speedY);

            robot.getGlyphLift().setLiftMotorPower(liftMotorPower);
        }
    }
}

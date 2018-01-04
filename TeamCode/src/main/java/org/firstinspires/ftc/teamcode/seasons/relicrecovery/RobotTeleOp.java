package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        robot.getRelicArm().initializeArm();

        waitForStart();

        double speedX;
        double speedY;
        double pivot;

        ElapsedTime intakeToggleTimer = new ElapsedTime();

        boolean isRaised = true;

        double liftMotorPower;
        double liftRotationMotorPower;

        while (opModeIsActive()) {
            speedX = -gamepad1.right_stick_x;
            speedY = gamepad1.right_stick_y;
            pivot = -gamepad1.left_stick_x;

            liftMotorPower = -gamepad2.left_stick_y;
            liftRotationMotorPower = -gamepad2.left_stick_x;

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
            if(gamepad1.left_bumper && intakeToggleTimer.milliseconds() > 500) {  //toggle if raised or lowered
                if(isRaised){
                    robot.getIntake().lowerIntake();
                    isRaised = false;

                } else if(!isRaised){
                    robot.getIntake().raiseIntake();
                    isRaised = true;
                }
                intakeToggleTimer.reset();
            }
            telemetry.addData("isRaised", isRaised);
            telemetry.update();
            // run intake in
            if(gamepad1.left_trigger > 0.1) {
                robot.getIntake().setIntakePower(-1);
            } else if(gamepad1.right_bumper){  // run intake out
                robot.getIntake().setIntakePower(1);
            } else {
                robot.getIntake().setIntakePower(0);
            }

            // intake linkage
            if(gamepad1.left_trigger > 0.1 || gamepad1.right_bumper){
                robot.getIntake().closeLinkage();
            }/** else if(robot.getIntake().isLinkClosed()){
                robot.getIntake().openLinkage();
            }*/

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
            if(gamepad2.dpad_up) {
                robot.getGlyphLift().setRotationMotorPosition(GlyphLift.RotationMotorPosition.UP);
            } else if(gamepad2.dpad_down) {
                robot.getGlyphLift().setRotationMotorPosition(GlyphLift.RotationMotorPosition.DOWN);
            } else {
                robot.getGlyphLift().setRotationMotorPower(liftRotationMotorPower);
            }

            // TRUE is Main Arm Control, False is Extension Control     RELIC ARM CONTROLS
            boolean mode = true;
            // mode switcher
            if(gamepad2.right_stick_button){
                if(mode = true){
                    mode = false;
                    telemetry.addData("Right Stick Mode", "Extension");
                } else {
                    mode = true;
                    telemetry.addData("Right Stick Mode", "Main Arm");
                }
                telemetry.update();
            }
            // control arm
            if(gamepad2.right_stick_x > 0){
                if(mode) { // Main Arm Control
                    robot.getRelicArm().setArmMainPower(gamepad2.right_stick_x);
                } else {    // Extension Control
                    robot.getRelicArm().setArmExtensionPower(gamepad2.right_stick_x);
                }
            }
            // relic gripper controls
            if(gamepad2.a){
                robot.getRelicArm().openGrip();
            } else if (gamepad2.b){
                robot.getRelicArm().closeGrip();
            }

            if(gamepad2.y) {
                robot.getGlyphLift().rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

//            telemetry.addData("Red Level", robot.getJewelKnocker().getRed());
//            telemetry.addData("Blue Level", robot.getJewelKnocker().getBlue());
//            telemetry.update();

            robot.getHDriveTrain().pivot(pivot);
            robot.getHDriveTrain().drive(speedX, speedY);

            robot.getGlyphLift().setLiftMotorPower(liftMotorPower);
        }
    }
}

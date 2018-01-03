package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;

/**
 * This class is the competition robot tele-op program.
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

        waitForStart();

        double speedX;
        double speedY;
        double pivot;

        double liftMotorPower;
        double liftRotationMotorPower;

        while (opModeIsActive()) {
            speedX = -gamepad1.right_stick_x;
            speedY = gamepad1.right_stick_y;
            pivot = -gamepad1.left_stick_x;

            liftMotorPower = -gamepad2.left_stick_y;
            liftRotationMotorPower = -gamepad2.left_stick_x;

            // slow down robot with right trigger
            if(gamepad1.right_trigger > 0) {
                speedY /= 3;
                pivot /= 3;
                speedX /= 2;
            }

            // Jewel arm control
            if (gamepad1.y){
                robot.getJewelKnocker().extendArm();
            } else {
                robot.getJewelKnocker().retractArm();
            }
            boolean isRaised = true;
            // intake raise/lower control
            if(gamepad1.left_bumper) {  //toggle if raised or lowered
                if(isRaised){
                    isRaised = false;
                    robot.getIntake().lowerIntake();
                } else if(!isRaised){
                    isRaised = true;
                    robot.getIntake().raiseIntake();
                }
            }
            if(gamepad1.left_trigger > 0.1){
                robot.getIntake().setIntakePower(1);
                robot.getIntake().openLinkage();
            }

            //if(gamepad1.left_trigger < 0.1 && gamepad1.)

            // close/open blue gripper
            if(gamepad2.right_trigger > 0.1) {
                robot.getGlyphLift().openBlueGripper();
            } else {
                robot.getGlyphLift().closeBlueGripper();
            }

            if(gamepad1.b){
                robot.getJewelKnocker().retractArm();
            }
            // TRUE is Main Arm Control, False is Extension Control
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

            if(gamepad2.b) {
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

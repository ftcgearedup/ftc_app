package org.firstinspires.ftc.teamcode.seasons.relicrecovery.trainingrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;

/**
 * This class is the competition robot tele-op program.
 */
@TeleOp(name = "TELEOP", group = "teleop")
public class TrainingTeleOp extends LinearOpMode {
    private TrainingRobot robot;

    private static final float JOYSTICK_DEADZONE = 0.2f;
    /**
            CONTROLS USED

            Right Stick   - Turning Head
            Left Stick - Movement + Turning
            Right Trigger - Slow Driving

     */

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TrainingRobot(this);

        gamepad1.setJoystickDeadzone(JOYSTICK_DEADZONE);
        gamepad2.setJoystickDeadzone(JOYSTICK_DEADZONE);

        waitForStart();

        
    }
}

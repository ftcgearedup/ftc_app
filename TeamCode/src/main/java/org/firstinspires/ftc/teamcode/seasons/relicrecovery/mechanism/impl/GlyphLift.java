package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

import java.io.File;

/**
 * The glyph lift mechanism collects glyphs with two grippers and is able to place them in the cryptobox.
 */

public class GlyphLift implements IMechanism {

    private JSONConfigOptions optionsMap = new JSONConfigOptions();

    private static final double MAX_LIFT_MOTOR_POWER_UP = 0.4;
    private static final double MAX_LIFT_MOTOR_POWER_DOWN = 0.9;

    private OpMode opMode;

    private DcMotor liftMotor;
    private DcMotor glyphIntakeMotor;
    private TouchSensor touchSensor;
    private ColorSensor colorSensor;

    /**
     * Construct a new {@link GlyphLift} with a reference to the utilizing robot.
     *
     * @param robot the robot using this glyph lift
     */
    public GlyphLift(Robot robot) {

        DcMotorSimple.Direction liftMotorDir = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction intakeMotorDir = DcMotorSimple.Direction.REVERSE;

        if(optionsMap.retrieveData("glyphLiftIsLiftReversed").getAsBoolean()){
            liftMotorDir = DcMotorSimple.Direction.REVERSE;
        } else {
            liftMotorDir = DcMotorSimple.Direction.FORWARD;
        }

        if(optionsMap.retrieveData("glyphLiftIsIntakeReversed").getAsBoolean()){
            intakeMotorDir = DcMotorSimple.Direction.REVERSE;
        } else {
            intakeMotorDir = DcMotorSimple.Direction.FORWARD;
        }



        this.opMode = robot.getCurrentOpMode();
        HardwareMap hwMap = opMode.hardwareMap;

        this.liftMotor = hwMap.dcMotor.get("lift");
        this.glyphIntakeMotor = hwMap.dcMotor.get("gm");
        this.touchSensor = hwMap.touchSensor.get("gts");
        this.colorSensor = hwMap.colorSensor.get("gcs");

        // reverse lift motor
        liftMotor.setDirection(liftMotorDir);

        // reverse glyph intake motor
        glyphIntakeMotor.setDirection(intakeMotorDir);

        // brake both motors
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    public TouchSensor getTouchSensor() {
        return touchSensor;
    }

    /**
     * This method sets the power of the intake motor.
     * The intake motor controls if the intake runs inward or outward.
     *
     *@param power the power to run the intake motor in a range of 1.0 to -1.0
     *             Negative values run the intake inward, positive
     */
    public void setGlyphIntakeMotorPower(double power) {
        glyphIntakeMotor.setPower(power);
    }

    /**
     * This method sets the power of the lift motor.
     * The lift motor is the motor that raises and lowers the linear slides.
     *
     * @param power the power to run the lift motor in a range of 1.0 to -1.0.
     *              Negative values lower the lift and, likewise, positive values raise the lift.
     */
    public void setLiftMotorPower(double power) {
        liftMotor.setPower(-power * (power < 0 ? MAX_LIFT_MOTOR_POWER_UP : MAX_LIFT_MOTOR_POWER_DOWN));
    }
}

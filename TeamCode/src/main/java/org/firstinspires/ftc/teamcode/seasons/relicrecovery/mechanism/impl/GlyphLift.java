package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * The glyph lift mechanism collects glyphs with two grippers and is able to place them in the cryptobox.
 */

public class GlyphLift implements IMechanism {

    private JSONConfigOptions optionsMap = new JSONConfigOptions();

    private static final double MAX_LIFT_MOTOR_POWER_UP = 0.4;
    private static final double MAX_LIFT_MOTOR_POWER_DOWN = 0.9;

    private OpMode opMode;

    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;
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

        this.liftMotorLeft = hwMap.dcMotor.get("liftml");
        this.liftMotorRight = hwMap.dcMotor.get("liftmr");
        this.glyphIntakeMotor = hwMap.dcMotor.get("gm");
        this.touchSensor = hwMap.touchSensor.get("gts");
        this.colorSensor = hwMap.colorSensor.get("gcs");

        // reverse lift motor
        liftMotorLeft.setDirection(liftMotorDir);

        // reverse glyph intake motor
        glyphIntakeMotor.setDirection(intakeMotorDir);

        // brake both motors
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
     * This method sets the power of the Left lift motor.
     * The Left lift motor is the motor that raises and lowers the linear slides.
     * the right lift motor is one that assists the Left lift motor, and its direction must be
     *  reversed because it is facing the opposite direction.
     *
     * @param power the power to run the Left lift motor in a range of 1.0 to -1.0.
     *              Negative values lower the lift and, likewise, positive values raise the lift.
     */
    public void setLiftMotorPower(double power) {
        double powerCoefficient = (power < 0 ? MAX_LIFT_MOTOR_POWER_UP : MAX_LIFT_MOTOR_POWER_DOWN);

        liftMotorLeft.setPower(-power * powerCoefficient);
        liftMotorRight.setPower(power * powerCoefficient);
    }
}

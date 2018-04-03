package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * The glyph lift mechanism collects glyphs with two grippers and is able to place them in the cryptobox.
 */

public class GlyphLift implements IMechanism {

    private JSONConfigOptions optionsMap;

    public final double MAX_LIFT_MOTOR_POWER_UP;
    public final double MAX_LIFT_MOTOR_POWER_DOWN;

    private final int LIFT_RAISED_POSITION;
    private final int GLYPH_EJECT_POSITION;
    private final int LIFT_MAX_ENCODER_POSITION;

    private OpMode opMode;

    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;

    private DcMotor leftGlyphIntakeMotor;
    private DcMotor rightGlyphIntakeMotor;

    private TouchSensor touchSensor;
    private ColorSensor colorSensor;

    private TouchSensor glyphTouchSensor;
    private DigitalChannel liftTouchSensor;

    /**
     * Construct a new {@link GlyphLift} with a reference to the utilizing robot.
     *
     * @param robot the robot using this glyph lift
     */
    public GlyphLift(Robot robot) {
        this.optionsMap = ((RelicRecoveryRobot)robot).getOptionsMap();
        LIFT_RAISED_POSITION = optionsMap.retrieveAsInt("glyphLiftLiftRaisedPosition");

        MAX_LIFT_MOTOR_POWER_UP = optionsMap.retrieveAsDouble("glyphLiftMotorPowerUp");
        GLYPH_EJECT_POSITION = optionsMap.retrieveAsInt("glyphLiftGlyphEjectPosition");
        MAX_LIFT_MOTOR_POWER_DOWN = optionsMap.retrieveAsDouble("glyphLiftMotorPowerDown");
        LIFT_MAX_ENCODER_POSITION = optionsMap.retrieveAsInt("glyphLiftMaxEncoderPosition");

        DcMotorSimple.Direction liftMotorDir;
        DcMotorSimple.Direction intakeMotorDir;

        if(optionsMap.retrieveAsBoolean("glyphLiftIsLiftReversed")){
            liftMotorDir = DcMotorSimple.Direction.REVERSE;
        } else {
            liftMotorDir = DcMotorSimple.Direction.FORWARD;
        }

        if(optionsMap.retrieveAsBoolean("glyphLiftIsIntakeReversed")){
            intakeMotorDir = DcMotorSimple.Direction.REVERSE;
        } else {
            intakeMotorDir = DcMotorSimple.Direction.FORWARD;
        }

        this.opMode = robot.getCurrentOpMode();
        HardwareMap hwMap = opMode.hardwareMap;

        this.liftMotorLeft = hwMap.dcMotor.get("liftml");
        this.liftMotorRight = hwMap.dcMotor.get("liftmr");

        this.leftGlyphIntakeMotor = hwMap.dcMotor.get("lgm");
        this.rightGlyphIntakeMotor = hwMap.dcMotor.get("rgm");

        this.touchSensor = hwMap.touchSensor.get("gts");
        this.colorSensor = hwMap.colorSensor.get("gcs");

        // reverse lift motor
        liftMotorLeft.setDirection(liftMotorDir);

        // reverse glyph intake motor
        leftGlyphIntakeMotor.setDirection(intakeMotorDir);
        rightGlyphIntakeMotor.setDirection(intakeMotorDir);

        // run using encoder
        leftGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // brake both motors
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse left and right lift motor
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Get the glyph lift color sensor
     *
     * @return the glyph lift color sensor
     */
    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    /**
     * Get the glyph touch sensor
     *
     * @return get the glyph touch sensor
     */
    public TouchSensor getGlyphTouchSensor() {
        return glyphTouchSensor;
    }

    /**
     * Get the lift touch sensor
     *
     * @return the lift touch sensor
     */
    public DigitalChannel getLiftTouchSensor() {
        return liftTouchSensor;
    }

    public int getLiftLeftMotorPosition() {
        return liftMotorLeft.getCurrentPosition();
    }

    public int getLiftRightMotorPosition() {
        return liftMotorRight.getCurrentPosition();
    }

    /**
     *
     * @return
     */
    public boolean isLiftTouchSensorPressed() {
        // the lift touch sensor value is inverted
        return !liftTouchSensor.getState();
    }

    private void setLiftMotorsPosition(int targetPosition, double speed) {
        // run to position for both motors
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set lift motors target positions
        liftMotorLeft.setTargetPosition(targetPosition);
        liftMotorRight.setTargetPosition(targetPosition);

        // set lift motors power
        liftMotorLeft.setPower(speed);
        liftMotorRight.setPower(speed);
    }

    /**
     * Run the intake in reverse in order to eject a glyph
     */
    public void ejectGlyph() {

        leftGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftGlyphIntakeMotor.setTargetPosition(GLYPH_EJECT_POSITION);
        rightGlyphIntakeMotor.setTargetPosition(-GLYPH_EJECT_POSITION);

        leftGlyphIntakeMotor.setPower(1.0);
        rightGlyphIntakeMotor.setPower(1.0);
    }

    /**
     * Raise the glyph lift
     */
    public void raiseGlyphLift() {
        setLiftMotorsPosition(LIFT_RAISED_POSITION, MAX_LIFT_MOTOR_POWER_UP);
    }

    /**
     * Return if the glyph lift is busy (i.e. currently lowering or raising).
     * The lift is busy after {@link #raiseGlyphLift()} is called.
     *
     * @return if the glyph lift is current running (lowering or raising)
     */
    public boolean isGlyphLiftBusy() {
        return liftMotorLeft.isBusy() && liftMotorRight.isBusy();
    }

    /**
     * This method sets the power of the intake motor.
     * The intake motor controls if the intake runs inward or outward.
     *
     *@param power the power to run the intake motor in a range of 1.0 to -1.0
     *             Negative values run the intake inward, positive runs it outward
     */
    public void setGlyphIntakeMotorPower(double power) {
        leftGlyphIntakeMotor.setPower(power);
        rightGlyphIntakeMotor.setPower(-power);
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
        double powerCoefficient = (power < 0 ? MAX_LIFT_MOTOR_POWER_DOWN : MAX_LIFT_MOTOR_POWER_UP);

        // set lift motors mode to run using encoders
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // if lift touch sensor is pressed and desired power is in reverse, stop the lift motors
        if (isLiftTouchSensorPressed() && power < 0) {
            powerCoefficient = 0;
        }

        // stop if any of the two lift motors exceeded their max position and the direction is up
        if((liftMotorLeft.getCurrentPosition() >= LIFT_MAX_ENCODER_POSITION
                || liftMotorRight.getCurrentPosition() >= LIFT_MAX_ENCODER_POSITION) && power > 0) {
            powerCoefficient = 0;
        }

        liftMotorLeft.setPower(power * powerCoefficient);
        liftMotorRight.setPower(power * powerCoefficient);
    }
}

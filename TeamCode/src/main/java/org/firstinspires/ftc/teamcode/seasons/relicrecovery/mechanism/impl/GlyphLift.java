package org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    public final double maxLiftMotorPowerUp;
    public final double maxLiftMotorPowerDown;

    private final int liftRaisedPosition;
    private final int glyphEjectPosition;
    private final int liftMaxEncoderPosition;

    private final double intakeLeftFullOpenPosition;
    private final double intakeRightFullOpenPosition;
    private final double intakeLeftHalfwayOpenPosition;
    private final double intakeRightHalfwayOpenPosition;
    private final double intakeLeftGripPosition;
    private final double intakeRightGripPosition;
    private final double intakeLeftInitializedPosition;
    private final double intakeRightInitializedPosition;

    private OpMode opMode;

    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;

    private DcMotor leftGlyphIntakeMotor;
    private DcMotor rightGlyphIntakeMotor;

    public Servo intakeArmLeft;
    public Servo intakeArmRight;

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
        liftRaisedPosition = optionsMap.retrieveAsInt("glyphLiftLiftRaisedPosition");

        maxLiftMotorPowerUp = optionsMap.retrieveAsDouble("glyphLiftMotorPowerUp");
        glyphEjectPosition = optionsMap.retrieveAsInt("glyphLiftGlyphEjectPosition");
        maxLiftMotorPowerDown = optionsMap.retrieveAsDouble("glyphLiftMotorPowerDown");
        liftMaxEncoderPosition = optionsMap.retrieveAsInt("glyphLiftMaxEncoderPosition");

        intakeLeftInitializedPosition = optionsMap.retrieveAsDouble("intakeLeftInitializedPosition");
        intakeLeftFullOpenPosition = optionsMap.retrieveAsDouble("intakeLeftFullOpenPosition");
        intakeLeftGripPosition = optionsMap.retrieveAsDouble("intakeLeftGripPosition");
        intakeLeftHalfwayOpenPosition = optionsMap.retrieveAsDouble("intakeLeftHalfwayOpenPosition");

        intakeRightInitializedPosition = optionsMap.retrieveAsDouble("intakeRightInitializedPosition");
        intakeRightFullOpenPosition = optionsMap.retrieveAsDouble("intakeRightFullOpenPosition");
        intakeRightGripPosition = optionsMap.retrieveAsDouble("intakeRightGripPosition");
        intakeRightHalfwayOpenPosition = optionsMap.retrieveAsDouble("intakeRightHalfwayOpenPosition");

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

        this.intakeArmLeft = hwMap.servo.get("ial");
        this.intakeArmRight = hwMap.servo.get("iar");

        this.leftGlyphIntakeMotor = hwMap.dcMotor.get("lgm");
        this.rightGlyphIntakeMotor = hwMap.dcMotor.get("rgm");

        this.liftTouchSensor = hwMap.digitalChannel.get("lts");
        this.glyphTouchSensor = hwMap.touchSensor.get("gts");

        this.colorSensor = hwMap.colorSensor.get("gcs");

        // reverse lift motor
        liftMotorLeft.setDirection(liftMotorDir);

        // reverse glyph intake motor
        leftGlyphIntakeMotor.setDirection(intakeMotorDir);
        rightGlyphIntakeMotor.setDirection(intakeMotorDir);

        // run using encoder
        leftGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset lift motor encoders
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set lift motors to run using encoder
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
     * Return whether the lift touch sensor is pressed.
     *
     * @return if the lift touch sensor is pressed
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

        leftGlyphIntakeMotor.setTargetPosition(glyphEjectPosition);
        rightGlyphIntakeMotor.setTargetPosition(-glyphEjectPosition);

        leftGlyphIntakeMotor.setPower(1.0);
        rightGlyphIntakeMotor.setPower(1.0);
    }

    /**
     * Raise the glyph lift
     */
    public void raiseGlyphLift() {
        setLiftMotorsPosition(liftRaisedPosition, maxLiftMotorPowerUp);
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
        double powerCoefficient = (power < 0 ? maxLiftMotorPowerDown : maxLiftMotorPowerUp);

        // set lift motors mode to run using encoders
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // if lift touch sensor is pressed and desired power is in reverse, stop the lift motors
        if (isLiftTouchSensorPressed() && power < 0) {
            powerCoefficient = 0;
        }

        // stop if any of the two lift motors exceeded their max position and the direction is up
        if((liftMotorLeft.getCurrentPosition() >= liftMaxEncoderPosition
                || liftMotorRight.getCurrentPosition() >= liftMaxEncoderPosition) && power > 0) {
            powerCoefficient = 0;
        }

        liftMotorLeft.setPower(power * powerCoefficient);
        liftMotorRight.setPower(power * powerCoefficient);
    }

    /**
     * Closes Intake. Intake will retract in full.
     */
    public void setIntakeInitializePosition() {
        intakeArmLeft.setPosition(intakeLeftInitializedPosition);
        intakeArmRight.setPosition(intakeRightInitializedPosition);
    }

    /**
     * Sets intake position to Grip position. Intake will pull in Glyphs in this position.
     */
    public void setIntakeGripPosition() {
        intakeArmLeft.setPosition(intakeLeftGripPosition);
        intakeArmRight.setPosition(intakeRightGripPosition);
    }

    /**
     * Sets intake position to fully open position. The intake will extend all the way out.
     * This will hit the wheel cover unless lift is raised high enough.
     * Use {@link #setIntakeHalfOpenPosition()} for a 45° angle.
     */
    public void setIntakeFullyOpenPosition() {
        intakeArmLeft.setPosition(intakeLeftFullOpenPosition);
        intakeArmRight.setPosition(intakeRightFullOpenPosition);
    }

    /**
     * Sets intake position to the half open position.
     * The Intake will open just enough to not touch the wheel cover.
     * For all the way open, use {@link #setIntakeFullyOpenPosition()}.
     */
    public void setIntakeHalfOpenPosition() {
        opMode.telemetry.addData("json", "setting left halfway to " + intakeLeftHalfwayOpenPosition);
        opMode.telemetry.update();
        intakeArmLeft.setPosition(intakeLeftHalfwayOpenPosition);
        intakeArmRight.setPosition(intakeRightHalfwayOpenPosition);
    }

    /**
     * Spin both Glyph Lift wheels in the same direction.
     *
     * @param isLeft true is rotate to the left, false is rotate to the right
     * @param power power at which to rotate motors at
     */
    public void spinWheelsInDirection(boolean isLeft, double power){
        leftGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPos = leftGlyphIntakeMotor.getTargetPosition() +
                optionsMap.retrieveAsInt("autonomousRotatePosition");

        if(isLeft){
            leftGlyphIntakeMotor.setTargetPosition(targetPos);
            rightGlyphIntakeMotor.setTargetPosition(targetPos);
        } else {
            leftGlyphIntakeMotor.setTargetPosition(-targetPos);
            rightGlyphIntakeMotor.setTargetPosition(-targetPos);
        }

        leftGlyphIntakeMotor.setPower(power);
        rightGlyphIntakeMotor.setPower(power);

        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        while(leftGlyphIntakeMotor.isBusy() && rightGlyphIntakeMotor.isBusy()){
            linearOpMode.idle();
        }

        leftGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

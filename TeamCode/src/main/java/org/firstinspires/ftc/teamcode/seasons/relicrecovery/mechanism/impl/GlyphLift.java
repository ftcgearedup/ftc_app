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

    private JSONConfigOptions optionsMap = new JSONConfigOptions("options.json");

    private final double MAX_LIFT_MOTOR_POWER_UP;
    private final double MAX_LIFT_MOTOR_POWER_DOWN;

    private final int LIFT_RAISED_POSITION = 850;
    private final int GLYPH_EJECT_POSITON = 3000;

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
        MAX_LIFT_MOTOR_POWER_UP = optionsMap.retrieveAsDouble("glyphLiftMotorPowerUp");
        MAX_LIFT_MOTOR_POWER_DOWN = optionsMap.retrieveAsDouble("glyphLiftMotorPowerDown");

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
        this.glyphIntakeMotor = hwMap.dcMotor.get("gm");
        this.touchSensor = hwMap.touchSensor.get("gts");
        this.colorSensor = hwMap.colorSensor.get("gcs");

        // reverse lift motor
        liftMotorLeft.setDirection(liftMotorDir);

        // reverse glyph intake motor
        glyphIntakeMotor.setDirection(intakeMotorDir);

        // run using encoder
        glyphIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // brake both motors
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse left and right lift motor
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    public TouchSensor getTouchSensor() {
        return touchSensor;
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
     *
     */
    public void ejectGlyph() {
        glyphIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphIntakeMotor.setTargetPosition(GLYPH_EJECT_POSITON);
        glyphIntakeMotor.setPower(1.0);
    }

    /**
     *
     */
    public void raiseGlyphLift() {
        setLiftMotorsPosition(LIFT_RAISED_POSITION, MAX_LIFT_MOTOR_POWER_UP);
    }

    /**
     *
     */
    public void lowerGlyphLift() {
        setLiftMotorsPosition(-LIFT_RAISED_POSITION, MAX_LIFT_MOTOR_POWER_DOWN);
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
        double powerCoefficient = (power < 0 ? MAX_LIFT_MOTOR_POWER_DOWN : MAX_LIFT_MOTOR_POWER_UP);

        liftMotorLeft.setPower(power * powerCoefficient);
        liftMotorRight.setPower(power * powerCoefficient);
    }
}

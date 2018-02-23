package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4AbstractSensor;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4MB1040;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4MB1043;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.JewelKnocker;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.RelicArm;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * This class represents the Relic Recovery robot.
 */
public class RelicRecoveryRobot extends Robot {
    private final HDriveTrain hDriveTrain;
    private final VisionHelper visionHelper;

    private final GlyphLift glyphLift;
    private final JewelKnocker jewelKnocker;
    private RelicArm relicArm;

    private MaxSonarEZ4AbstractSensor leftRangeSensor;
    private MaxSonarEZ4AbstractSensor rightRangeSensor;
    private MaxSonarEZ4AbstractSensor frontRangeSensor;

    private final JSONConfigOptions optionsMap;

    /**
     * Construct a new Relic Recovery robot, with an op-mode that is using this robot.
     *
     * @param opMode the op-mode that this robot is using.
     * @param configOptions the {@link JSONConfigOptions} instance to use with this robot object
     */
    public RelicRecoveryRobot(OpMode opMode) {
        super(opMode);

        this.optionsMap = new JSONConfigOptions("options.json");

        DcMotor.Direction rightMotorDirection;
        DcMotor.Direction leftMotorDirection;

        if(optionsMap.retrieveAsBoolean("isRightMotorReversed")){
            rightMotorDirection = DcMotor.Direction.REVERSE;
        } else {
            rightMotorDirection = DcMotor.Direction.FORWARD;
        }

        if(optionsMap.retrieveAsBoolean("isLeftMotorReversed")){
            leftMotorDirection = DcMotor.Direction.REVERSE;
        } else {
            leftMotorDirection = DcMotor.Direction.FORWARD;
        }

        double wheelDiameter = optionsMap.retrieveAsDouble("wheelDiameter");
        double wheelGearRatioIn = optionsMap.retrieveAsDouble("wheelGearRatioIn");
        double wheelGearRatioOut = optionsMap.retrieveAsDouble("wheelGearRatioOut");

        this.hDriveTrain = new HDriveTrain.Builder(this)
                .setLeftMotorDirection(leftMotorDirection)
                .setRightMotorDirection(rightMotorDirection)
                .setWheelDiameterInches(wheelDiameter)
                .setInsideWheelGearingRatio(wheelGearRatioIn)
                .setOutsideWheelGearingRatio(wheelGearRatioOut)
                .build();

        this.glyphLift = new GlyphLift(this);
        this.visionHelper = new VisionHelper(this);
        this.jewelKnocker = new JewelKnocker(this);
//        this.relicArm = new RelicArm(this);

        this.frontRangeSensor = new MaxSonarEZ4MB1040(this, "frs");

        this.rightRangeSensor = new MaxSonarEZ4MB1043(this, "rrs");
        this.leftRangeSensor = new MaxSonarEZ4MB1043(this, "lrs");
    }

    public HDriveTrain getHDriveTrain() {
        return hDriveTrain;
    }

    public GlyphLift getGlyphLift() {
        return glyphLift;
    }

    public VisionHelper getVisionHelper() {
        return visionHelper;
    }

    public JewelKnocker getJewelKnocker() {
        return jewelKnocker;
    }

    public JSONConfigOptions getOptionsMap() {
        return optionsMap;
    }

    public RelicArm getRelicArm() {
        return relicArm;
    }

    public MaxSonarEZ4AbstractSensor getLeftRangeSensor() {
        return leftRangeSensor;
    }

    public MaxSonarEZ4AbstractSensor getRightRangeSensor() {
        return rightRangeSensor;
    }

    public MaxSonarEZ4AbstractSensor getFrontRangeSensor() {
        return frontRangeSensor;
    }
}


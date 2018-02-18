package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4Sensor;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.JewelKnocker;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.RelicArm;

import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

import java.io.File;

/**
 * This class represents the Relic Recovery robot.
 */
public class RelicRecoveryRobot extends Robot {
    private final HDriveTrain hDriveTrain;
    private final VisionHelper visionHelper;

    private final GlyphLift glyphLift;
    private final JewelKnocker jewelKnocker;
    private RelicArm relicArm;

    private MaxSonarEZ4Sensor leftRangeSensor;
    private MaxSonarEZ4Sensor rightRangeSensor;
    private MaxSonarEZ4Sensor frontRangeSensor;

    private final JSONConfigOptions optionsMap;

    /**
     * Construct a new Relic Recovery robot, with an op-mode that is using this robot.
     *
     * @param opMode the op-mode that this robot is using.
     */
    public RelicRecoveryRobot(OpMode opMode) {
        super(opMode);

        this.optionsMap = new JSONConfigOptions();

        optionsMap.parseFile(new File(AppUtil.FIRST_FOLDER + "/options.json"));

        boolean isRightMotorReversed = optionsMap.retrieveData("isRightMotorReversed").getAsBoolean();
        DcMotor.Direction rightMotorDirection;
        if(isRightMotorReversed){
            rightMotorDirection = DcMotorSimple.Direction.REVERSE;
        } else {
            rightMotorDirection = DcMotorSimple.Direction.FORWARD;
        }

        double wheelDiameter = optionsMap.retrieveData("wheelDiameter").getAsDouble();
        double wheelGearRatioIn = optionsMap.retrieveData("wheelGearRatioIn").getAsDouble();
        double wheelGearRatioOut = optionsMap.retrieveData("wheelGearRatioOut").getAsDouble();


        this.hDriveTrain = new HDriveTrain.Builder(this)
                .setRightMotorDirection(rightMotorDirection)
                .setWheelDiameterInches(wheelDiameter)
                .setInsideWheelGearingRatio(wheelGearRatioIn)
                .setOutsideWheelGearingRatio(wheelGearRatioOut)
                .build();

        this.glyphLift = new GlyphLift(this);
        this.visionHelper = new VisionHelper(this);
        this.jewelKnocker = new JewelKnocker(this);
//        this.relicArm = new RelicArm(this);

        this.frontRangeSensor = new MaxSonarEZ4Sensor(this, "frs");
//        this.rightRangeSensor = new MaxSonarEZ4Sensor(this, "rrs");
//        this.leftRangeSensor = new MaxSonarEZ4Sensor(this, "lrs");

        visionHelper.initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);
        //visionHelper.initializeOpenCV();
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

    public MaxSonarEZ4Sensor getLeftRangeSensor() {
        return leftRangeSensor;
    }

    public MaxSonarEZ4Sensor getRightRangeSensor() {
        return rightRangeSensor;
    }

    public MaxSonarEZ4Sensor getFrontRangeSensor() {
        return frontRangeSensor;
    }
}


package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.google.gson.JsonPrimitive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.Intake;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.JewelKnocker;
import org.firstinspires.ftc.teamcode.seasons.resq.DriveFunctions;

import java.io.File;
import java.util.Map;

/**
 * This class represents the Relic Recovery robot.
 */
public class RelicRecoveryRobot extends Robot {
    private final HDriveTrain hDriveTrain;
    private final VisionHelper visionHelper;

    private final GlyphLift glyphLift;
    private final Intake intake;
    private final JewelKnocker jewelKnocker;

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
        this.intake = new Intake(this);
        this.visionHelper = new VisionHelper(this);
        this.jewelKnocker = new JewelKnocker(this);

        //visionHelper.initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);
        //visionHelper.initializeOpenCV();
    }

    public HDriveTrain getHDriveTrain() {
        return hDriveTrain;
    }

    public GlyphLift getGlyphLift() {
        return glyphLift;
    }

    public Intake getIntake() {
        return intake;
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

}


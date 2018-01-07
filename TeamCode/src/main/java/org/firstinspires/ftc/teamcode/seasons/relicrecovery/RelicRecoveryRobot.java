package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4Sensor;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.Intake;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.JewelKnocker;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.RelicArm;

/**
 * This class represents the Relic Recovery robot.
 */
public class RelicRecoveryRobot extends Robot {
    private final HDriveTrain hDriveTrain;
    private final VisionHelper visionHelper;

    private final GlyphLift glyphLift;
    private final Intake intake;
    private final JewelKnocker jewelKnocker;
    private final RelicArm relicArm;

    private final MaxSonarEZ4Sensor leftRangeSensor;
    private final MaxSonarEZ4Sensor rightRangeSensor;
    private final MaxSonarEZ4Sensor frontRangeSensor;

    /**
     * Construct a new Relic Recovery robot, with an op-mode that is using this robot.
     *
     * @param opMode the op-mode that this robot is using.
     */
    public RelicRecoveryRobot(OpMode opMode) {
        super(opMode);

        this.hDriveTrain = new HDriveTrain.Builder(this)
                .setRightMotorDirection(DcMotor.Direction.REVERSE)
                .setWheelDiameterInches(4)
                .setInsideWheelGearingRatio(1.0)
                .setOutsideWheelGearingRatio(1.5)
                .build();

        this.glyphLift = new GlyphLift(this);
        this.intake = new Intake(this);
        this.visionHelper = new VisionHelper(this);
        this.jewelKnocker = new JewelKnocker(this);
        this.relicArm = new RelicArm(this);

        this.frontRangeSensor = new MaxSonarEZ4Sensor(this, "frs");
        this.rightRangeSensor = new MaxSonarEZ4Sensor(this, "rrs");
        this.leftRangeSensor = new MaxSonarEZ4Sensor(this, "lrs");

        visionHelper.initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);
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

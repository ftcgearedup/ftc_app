package org.firstinspires.ftc.teamcode.seasons.relicrecovery.trainingrobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.GlyphLift;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.Intake;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.JewelKnocker;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.mechanism.impl.RelicArm;

/**
 * This class represents the Relic Recovery robot.
 */
public class TrainingRobot extends Robot {
    private final HDriveTrain hDriveTrain;

    private final Head head;

    /**
     * Construct a new Relic Recovery robot, with an op-mode that is using this robot.
     *
     * @param opMode the op-mode that this robot is using.
     */
    public TrainingRobot(OpMode opMode) {
        super(opMode);

        this.hDriveTrain = new HDriveTrain.Builder(this)
                .setLeftMotorDirection(DcMotor.Direction.REVERSE)
                .setWheelDiameterInches(4)
                .setInsideWheelGearingRatio(1.0)
                .setOutsideWheelGearingRatio(1.5)
                .build();

        this.head = new Head(this);
    }

    public HDriveTrain getHDriveTrain() {
        return hDriveTrain;
    }

    public Head getHead() {
        return head;
    }

}

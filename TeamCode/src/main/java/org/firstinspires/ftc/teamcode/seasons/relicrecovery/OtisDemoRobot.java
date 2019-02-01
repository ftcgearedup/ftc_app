package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4AbstractSensor;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4MB1040;
import org.firstinspires.ftc.teamcode.mechanism.impl.MaxSonarEZ4MB1043;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;

import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * This class represents the Relic Recovery robot.
 */
public class OtisDemoRobot extends Robot {
    private final HDriveTrain hDriveTrain;


    /**
     * Construct a new Relic Recovery robot, with an op-mode that is using this robot.
     *
     * @param opMode the op-mode that this robot is using.
     */
    public OtisDemoRobot(OpMode opMode) {
        super(opMode);



        this.hDriveTrain = new HDriveTrain.Builder(this)
                .setLeftMotorDirection(DcMotor.Direction.REVERSE)
                .setRightMotorDirection(DcMotor.Direction.FORWARD)
                .setWheelDiameterInches(4)
                .setInsideWheelGearingRatio(1)
                .setOutsideWheelGearingRatio(1)
                .build();


        opMode.telemetry.addData(">", "done initializing");
        opMode.telemetry.update();
    }

    public HDriveTrain getHDriveTrain() {
        return hDriveTrain;
    }


}


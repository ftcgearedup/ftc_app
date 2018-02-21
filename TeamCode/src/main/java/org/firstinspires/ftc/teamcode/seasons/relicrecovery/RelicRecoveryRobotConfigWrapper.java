package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

public class RelicRecoveryRobotConfigWrapper {
    private RelicRecoveryRobot robot;
    private HDriveTrain hDriveTrain;
    private JSONConfigOptions configOptions;

    public RelicRecoveryRobotConfigWrapper(RelicRecoveryRobot robot, JSONConfigOptions configOptions) {
        this.robot = robot;
        this.configOptions = configOptions;
    }

    private DcMotor.Direction getMotorDirection(String key) {
        DcMotor.Direction direction;

        if(configOptions.retrieveData(key).getAsBoolean()){
            direction = DcMotor.Direction.REVERSE;
        } else {
            direction = DcMotor.Direction.FORWARD;
        }

        return direction;
    }

    public HDriveTrain getHDriveTrain() {
        DcMotor.Direction rightMotorDirection;
        DcMotor.Direction leftMotorDirection;

        rightMotorDirection = getMotorDirection("isRightMotorReversed");
        leftMotorDirection = getMotorDirection("isLeftMotorReversed");

        double wheelDiameter = configOptions.retrieveData("wheelDiameter").getAsDouble();
        double wheelGearRatioIn = configOptions.retrieveData("wheelGearRatioIn").getAsDouble();
        double wheelGearRatioOut = configOptions.retrieveData("wheelGearRatioOut").getAsDouble();

        this.hDriveTrain = new HDriveTrain.Builder(robot)
                .setLeftMotorDirection(leftMotorDirection)
                .setRightMotorDirection(rightMotorDirection)
                .setWheelDiameterInches(wheelDiameter)
                .setInsideWheelGearingRatio(wheelGearRatioIn)
                .setOutsideWheelGearingRatio(wheelGearRatioOut)
                .build();

        return hDriveTrain;
    }
}

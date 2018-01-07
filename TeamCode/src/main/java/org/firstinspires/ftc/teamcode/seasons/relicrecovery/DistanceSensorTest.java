package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;

/**
 * Created by ftc6347 on 1/7/18.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private DistanceSensorDriveAlgorithm distanceSensorDriveAlgorithm;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);
        distanceSensorDriveAlgorithm = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getFrontRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.FRONT);

        while(!isStarted() && !opModeIsActive()) {
            telemetry.addData("front", robot.getFrontRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("left", robot.getLeftRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("right", robot.getRightRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        distanceSensorDriveAlgorithm.driveToDistance(7, 0.5, false);
    }
}

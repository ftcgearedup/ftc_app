package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;

/**
 */
@Autonomous(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {
    private RelicRecoveryRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);

        while(!isStarted() && !opModeIsActive()) {
            telemetry.addData("front", robot.getFrontRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("left", robot.getLeftRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("right", robot.getRightRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}

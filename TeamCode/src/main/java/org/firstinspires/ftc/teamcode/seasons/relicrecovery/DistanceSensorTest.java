package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;

/**
 */
@Autonomous(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private DistanceSensorDriveAlgorithm rightDistanceSensorDrive;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RelicRecoveryRobot(this);

        this.rightDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getRightRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.RIGHT);

        this.bno055IMUWrapper = new BNO055IMUWrapper(robot);
        this.gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);
        bno055IMUWrapper.startIntegration();

        while(!isStarted() && !opModeIsActive()) {
            telemetry.addData("front", robot.getFrontRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("left", robot.getLeftRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.addData("right", robot.getRightRangeSensor().getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        do {
            gyroPivotAlgorithm.pivot(0.1, 0, true, true);
            rightDistanceSensorDrive.driveToDistance(25, 1.0, true);
        } while(rightDistanceSensorDrive.isAlgorithmBusy());

        //rightDistanceSensorDrive.driveToDistance(25, 1.0, false);
    }
}

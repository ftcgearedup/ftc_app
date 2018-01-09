package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;

/**
 * Created by ftc6347 on 1/8/18.
 */

@Autonomous(name = "Gyro Drive Test")
public class GyroDriveTest extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RelicRecoveryRobot(this);

        bno055IMUWrapper = new BNO055IMUWrapper(robot);
        gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);

        bno055IMUWrapper.startIntegration();

        waitForStart();

        gyroPivotAlgorithm.pivot(0.4, 180, true, false);

//        do {
//            robot.getHDriveTrain().directionalDrive(270, 0.5, 24, true);
//            gyroPivotAlgorithm.pivot(0.5, 90, true, true);
//        }
//        while(opModeIsActive() && Math.abs(90 - bno055IMUWrapper.getHeading()) > 0.5);
    }
}

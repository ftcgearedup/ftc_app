package org.firstinspires.ftc.teamcode.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;

/**
 * An implementation of {@link IGyroPivotAlgorithm}
 * utilizing an {@link BNO055IMUWrapper} as the gyroscopic sensor.
 */

public class BNO055IMUGyroPivotAlgorithm implements IGyroPivotAlgorithm {

    private BNO055IMUWrapper imu;
    private IDriveTrain driveTrain;

    private OpMode opMode;

    private double targetAngle;
    private double error;
    private double desiredSpeed;
    private boolean absolute;

    double previousTime = 0;
    double previousError = 0;
    double integral = 0;

    private static final double GYRO_DEGREE_THRESHOLD = 0.5;

    private static final double P_COEFF = 0.02;
    private static final double I_COEFF = 1.0;
    private static final double D_COEFF = 0;

    /**
     * Create a new instance of this algorithm implementation that will use the specified robot.
     *
     * @param robot the Robot instance using this gyro-pivot algorithm
     * @param imu the imu wrapper object to be used as the gyroscope sensor
     */
    public BNO055IMUGyroPivotAlgorithm(Robot robot, IDriveTrain driveTrain, BNO055IMUWrapper imu) {
        this.opMode = robot.getCurrentOpMode();
        this.driveTrain = driveTrain;

        this.imu = imu;
    }

    @Override
    public void pivot(double speed, double angle, boolean absolute, boolean nonBlocking) {
        this.desiredSpeed = speed;
        this.targetAngle = angle;
        this.absolute = absolute;

        if(nonBlocking || !(opMode instanceof LinearOpMode)) {
            pivotNonBlocking();
        } else {
            pivotBlocking();
        }
    }

    private double getError(double targetAngle, boolean absolute) {
        double heading = imu.getHeading();

        if(heading < targetAngle) heading += 360;
        double left = heading - targetAngle;

        if(left < 180) {
            return -left;
        } else {
            return 360 - left;
        }

        // compensate from robot's targetAngle to the zero degree
        // if(!absolute) {
        //    diff -= getError(0, false);
        // }
    }

    private void executionLoop() {
        double derivative;
        double timeDelta;
        double actualSpeed;

        timeDelta = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();

        integral += error * timeDelta;
        derivative = (error - previousError) / timeDelta;

        actualSpeed = (P_COEFF * error) + (I_COEFF * integral) + (D_COEFF * derivative);

        driveTrain.pivot(desiredSpeed * Range.clip(actualSpeed, -1, 1));

        opMode.telemetry.addData("Z axis difference from targetAngle", error);
        opMode.telemetry.update();
    }

    private void pivotBlocking() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        do {
            error = getError(targetAngle, absolute);

            executionLoop();
        } while(linearOpMode.opModeIsActive() && Math.abs(error) > GYRO_DEGREE_THRESHOLD);

        // when we're on target, stop the robot
        driveTrain.stopDriveMotors();
    }

    private void pivotNonBlocking() {
        error = getError(targetAngle, absolute);
        if(Math.abs(error) > GYRO_DEGREE_THRESHOLD) {
            executionLoop();
        }
    }
}

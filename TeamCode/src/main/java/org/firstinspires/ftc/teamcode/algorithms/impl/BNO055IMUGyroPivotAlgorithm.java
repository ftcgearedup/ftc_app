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

    private double previousTime = 0;
    private double previousError = 0;
    private double integral = 0;

    private static final double GYRO_DEGREE_THRESHOLD = 0.2;

    private static final double P_COEFF = 0.016;
    private static final double I_COEFF = 0.0000020; // 3
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
            // reset instance variables
            this.previousTime = System.currentTimeMillis();
            this.previousError = 0;
            this.integral = 0;

            pivotBlocking();
        }
    }

    public double getError(double targetAngle, boolean absolute) {
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
        double currentTime;

        currentTime = System.currentTimeMillis();

        timeDelta = currentTime - previousTime;
        previousTime = currentTime;

        integral += (Math.abs(error) * timeDelta);
        derivative = Math.copySign((error - previousError) / timeDelta, error);

        previousError = error;

        actualSpeed = (P_COEFF * error) + (I_COEFF * Math.copySign(integral, error)) + (D_COEFF * derivative);

        // speed is negative when error is positive because robot needs to turn counterclockwise
        driveTrain.pivot(-desiredSpeed * Range.clip(actualSpeed, -1, 1));

        opMode.telemetry.addData("Z axis difference from targetAngle", error);
        opMode.telemetry.addData("integral term", I_COEFF * integral);
        opMode.telemetry.addData("actual speed", actualSpeed);
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
        } else {
            // reset instance variables
            this.previousTime = System.currentTimeMillis();
            this.previousError = 0;
            this.integral = 0;
        }
    }
}

package org.firstinspires.ftc.teamcode.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * An implementation of {@link IGyroPivotAlgorithm}
 * utilizing an {@link BNO055IMUWrapper} as the gyroscopic sensor.
 */

public class BNO055IMUGyroPivotAlgorithm implements IGyroPivotAlgorithm {

    private BNO055IMUWrapper imu;
    private IDriveTrain driveTrain;

    private OpMode opMode;

    private JSONConfigOptions optionsMap;

    private double targetAngle;
    private double error;
    private double desiredSpeed;
    private boolean absolute;

    private double previousTime = 0;
    private double previousError = 0;
    private double integral = 0;

    private final double GYRO_DEGREE_THRESHOLD;

    private final double P_COEFF;
    private final double I_COEFF;
    private final double D_COEFF;

    /**
     * Create a new instance of this algorithm implementation that will use the specified robot.
     *
     * @param robot the Robot instance using this gyro-pivot algorithm
     * @param imu the imu wrapper object to be used as the gyroscope sensor
     */
    public BNO055IMUGyroPivotAlgorithm(Robot robot, IDriveTrain driveTrain, BNO055IMUWrapper imu) {
        this.opMode = robot.getCurrentOpMode();
        this.driveTrain = driveTrain;
        this.optionsMap = ((RelicRecoveryRobot)robot).getOptionsMap();

        // parse constants from configuration file
        this.GYRO_DEGREE_THRESHOLD = optionsMap.retrieveAsDouble("gyroPivotGyroDegreeThreshold");
        this.P_COEFF = optionsMap.retrieveAsDouble("gyroPivotPCoeff");
        this.I_COEFF = optionsMap.retrieveAsDouble("gyroPivotICoeff");
        this.D_COEFF = optionsMap.retrieveAsDouble("gyroPivotDCoeff");

        this.imu = imu;
    }

    @Override
    public boolean isAlgorithmBusy() {
        return Math.abs(error) > GYRO_DEGREE_THRESHOLD;
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

        RobotLog.dd("GYRO-PIVOT", "I: " + I_COEFF * Math.copySign(integral, error));

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
            driveTrain.pivot(0);
        }
    }
}

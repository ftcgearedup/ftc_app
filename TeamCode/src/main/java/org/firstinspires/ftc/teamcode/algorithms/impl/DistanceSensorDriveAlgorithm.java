package org.firstinspires.ftc.teamcode.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDirectionalDriveTrain;

/**
 * An algorithm that drives to a specified target currentDistance in inches
 * (driving in reverse if necessary) using a currentDistance sensor.
 * This algorithm supports driving forward and backward or left and right to
 * or from the target currentDistance using an {@link IDirectionalDriveTrain}
 *
 * @see DistanceSensor the interface for all currentDistance sensors
 */

public class DistanceSensorDriveAlgorithm {
    private OpMode opMode;

    private IDirectionalDriveTrain driveTrain;
    private DistanceSensor distanceSensor;

    private RobotSide sensorRobotSide;

    private double targetDistance;
    private double currentDistance;

    private double speedX;
    private double speedY;

    private static final double P_DRIVE_SPEED_COEFF = 0.03;
    private static final double DISTANCE_THRESHOLD = 0.5;

    /**
     * An enumeration type that represents the side of the robot the currentDistance sensor is located.
     */
    public enum RobotSide {
        FRONT(0, 1), BACK(0, -1), LEFT(-1, 0), RIGHT(1, 0);

        private int x;
        private int y;

        RobotSide(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Create a new instance of this algorithm with a reference to the robot,
     * a directional drive train, the currentDistance sensor, and the side of the robot
     * the currentDistance sensor is positioned.
     *
     * @param robot the robot utilizing this algorithm
     * @param distanceSensor the currentDistance sensor that detects the currentDistance to drive to
     * @param sensorRobotSide the side of the robot the currentDistance sensor is located
     */
    public DistanceSensorDriveAlgorithm(Robot robot, IDirectionalDriveTrain driveTrain,
                                        DistanceSensor distanceSensor,
                                        RobotSide sensorRobotSide) {
        this.opMode = robot.getCurrentOpMode();
        this.driveTrain = driveTrain;

        this.distanceSensor = distanceSensor;
        this.sensorRobotSide = sensorRobotSide;
    }

    /**
     *
     * @return
     */
    public boolean isAlgorithmBusy() {
        return Math.abs(targetDistance - currentDistance) > DISTANCE_THRESHOLD;
    }

    /**
     * Execute the algorithm to drive to {@code targetDistance} inches
     * at the specified speed using the currentDistance sensor. The algorithm will drive the
     * robot in reverse if {@code targetDistance} is greater than the detected robot currentDistance.
     *
     * @param targetDistance the currentDistance the currentDistance sensor should be from the target
     * @param speed the speed at which to drive
     * @param nonBlocking whether this method should block. If this method is called by a
     *                    non-{@link LinearOpMode}, this method will always be non-blocking, regardless
     *                    of the value for this parameter, due to the nature of non-{@link LinearOpMode}.
     */
    public void driveToDistance(double targetDistance, double speed, boolean nonBlocking) {
        this.targetDistance = targetDistance;

        if(nonBlocking || !(opMode instanceof LinearOpMode)) {
            driveToDistanceNonBlocking(speed);
        } else {
            driveToDistanceBlocking(speed);
        }
    }

    private void executionLoop(double speed) {
        speed *= Math.abs(targetDistance - currentDistance) * P_DRIVE_SPEED_COEFF;

        this.speedX = sensorRobotSide.x * speed;
        this.speedY = sensorRobotSide.y * speed;

        if(currentDistance > targetDistance) {
            driveTrain.drive(speedX, speedY);
        } else {
            // change both to negative for opposite direction
            driveTrain.drive(-speedX, -speedY);
        }
    }

    private void driveToDistanceBlocking(double speed) {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        do {
            this.currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
            executionLoop(speed);

            opMode.telemetry.addData("currentDistance", currentDistance);
            opMode.telemetry.addData("speed X", speedX);
            opMode.telemetry.addData("speed Y", speedY);
            opMode.telemetry.update();

        } while(linearOpMode.opModeIsActive() && isAlgorithmBusy());

        driveTrain.stopDriveMotors();
    }

    private void driveToDistanceNonBlocking(double speed) {
        this.currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        opMode.telemetry.addData("currentDistance", currentDistance);
        opMode.telemetry.addData("speed X", speedX);
        opMode.telemetry.addData("speed Y", speedY);
        opMode.telemetry.update();

        if(isAlgorithmBusy()) {
            executionLoop(speed);
        } else {
            driveTrain.stopDriveMotors();
        }
    }
}

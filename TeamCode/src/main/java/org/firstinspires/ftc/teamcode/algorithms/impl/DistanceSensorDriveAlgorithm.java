package org.firstinspires.ftc.teamcode.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.IDirectionalDriveTrain;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.RelicRecoveryRobot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.SortedSet;

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

    private double maxDistance;

    private ArrayList<Double> filterReadings;

    private int numReadings;
    private double lastReading;

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
        this.numReadings = ((RelicRecoveryRobot)robot).getOptionsMap().retrieveAsInt("rangeSensorFilterNumReadings");
        this.filterReadings = new ArrayList<>(numReadings);

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
    public void driveToDistance(double targetDistance, double maxDistance, double speed, boolean nonBlocking) {
        this.targetDistance = targetDistance;
        this.maxDistance = maxDistance;

        if(nonBlocking || !(opMode instanceof LinearOpMode)) {
            driveToDistanceNonBlocking(speed);
        } else {
            driveToDistanceBlocking(speed);
        }
    }

    public double lastFilteredReading() {
        currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if(filterReadings.size() < numReadings) {
            filterReadings.add(currentDistance);
        } else {
            lastReading = median(filterReadings);
            filterReadings.clear();
        }

        return lastReading;
    }

    private void executionLoop(double speed) {
        if(filterReadings.size() < numReadings) {
            filterReadings.add(currentDistance);
        } else {
            speed *= Math.abs(targetDistance - median(filterReadings)) * P_DRIVE_SPEED_COEFF;
            filterReadings.clear();

            this.speedX = sensorRobotSide.x * speed;
            this.speedY = sensorRobotSide.y * speed;

            if(currentDistance > targetDistance) {
                driveTrain.drive(speedX, speedY);
            } else {
                driveTrain.drive(-speedX, -speedY);
            }
        }
    }

    private double median(List<Double> list) {
        Collections.sort(list);
        int middle = list.size() / 2;

        if(list.size() % 2 == 1) {
            return list.get(middle);
        } else {
            return (list.get(middle - 1) + list.get(middle)) / 2;
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

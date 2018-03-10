package org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;

/**
 * This class provides methods that drive the robot forward while switching
 * between powering the left and right wheels quickly, making the robot "wiggle" as
 * it drives forward, which helps with grabbing glyphs in Autonomous.
 */
public class WiggleDriveAlgorithm {
    private HDriveTrain hDriveTrain;
    private OpMode opMode;

    /**
     * Constructor for the WiggleDriveAlgorithm class
     *
     * @param robot the RelicRecoveryRobot robot class
     * @param hDriveTrain the robot's HDriveTrain
     */
    public WiggleDriveAlgorithm(Robot robot, HDriveTrain hDriveTrain) {
        this.opMode = robot.getCurrentOpMode();
        this.hDriveTrain = hDriveTrain;
    }

    /**
     * Drive the robot forward with the "wiggle" motion
     *
     * @param speed the speed at which the robot should drive while "wiggling"
     * @param betweenMilliseconds time between wheel power switches (milliseconds)
     * @param milliseconds amount of milliseconds to drive for
     */
    public void drive(double speed, double betweenMilliseconds, double milliseconds) {
        ElapsedTime betweenTimer = new ElapsedTime();
        ElapsedTime driveTimer = new ElapsedTime();

        boolean driveLeft = false;

        if(opMode instanceof LinearOpMode) {
            LinearOpMode linearOpMode = (LinearOpMode)opMode;

            while(linearOpMode.opModeIsActive() && driveTimer.milliseconds() < milliseconds) {
                if(betweenTimer.milliseconds() > betweenMilliseconds) {
                    betweenTimer.reset();

                    driveLeft = !driveLeft;
                }

                if(driveLeft) {
                    hDriveTrain.setLeftMotorPower(0);
                    hDriveTrain.setRightMotorPower(speed);
                } else {
                    hDriveTrain.setRightMotorPower(0);
                    hDriveTrain.setLeftMotorPower(speed);
                }
            }
        }

        hDriveTrain.stopDriveMotors();
    }
}

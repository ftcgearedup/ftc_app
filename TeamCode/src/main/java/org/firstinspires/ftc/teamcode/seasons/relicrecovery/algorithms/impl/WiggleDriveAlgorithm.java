package org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;

/**
 *
 */
public class WiggleDriveAlgorithm {
    private HDriveTrain hDriveTrain;
    private OpMode opMode;

    /**
     *
     * @param robot
     * @param hDriveTrain
     */
    public WiggleDriveAlgorithm(Robot robot, HDriveTrain hDriveTrain) {
        this.opMode = robot.getCurrentOpMode();
        this.hDriveTrain = hDriveTrain;
    }

    /**
     *
     * @param speed
     * @param betweenMilliseconds
     * @param milliseconds
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

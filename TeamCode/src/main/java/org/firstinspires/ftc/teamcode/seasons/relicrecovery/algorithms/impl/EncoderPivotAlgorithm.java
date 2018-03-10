package org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.HDriveTrain;

public class EncoderPivotAlgorithm {
    private HDriveTrain hDriveTrain;
    private OpMode opMode;

    public EncoderPivotAlgorithm(Robot robot, HDriveTrain hDriveTrain) {
        this.opMode = robot.getCurrentOpMode();
        this.hDriveTrain = hDriveTrain;
    }

    public void encoderPivot(double speed, double encoderCounts) {
        if(opMode instanceof LinearOpMode) {
            LinearOpMode linearOpMode = (LinearOpMode)opMode;

            hDriveTrain.resetEncoders();

            while(linearOpMode.opModeIsActive()
                    && hDriveTrain.getLeftDriveMotor().getCurrentPosition() < encoderCounts
                    && hDriveTrain.getRightDriveMotor().getCurrentPosition() < encoderCounts) {
                hDriveTrain.pivot(speed);
            }

            hDriveTrain.stopDriveMotors();
        }
    }
}

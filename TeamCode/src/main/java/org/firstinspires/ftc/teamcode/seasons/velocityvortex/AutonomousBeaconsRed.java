package org.firstinspires.ftc.teamcode.seasons.velocityvortex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ftc6347 on 1/9/17.
 */
@Autonomous(name = "Beacons 1 RED", group = "1 beacons")
public class AutonomousBeaconsRed extends LinearOpModeBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        // reset drive encoders
        setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        autonomousInitLoop();

        // set target position for initial diagonal drive motion
        getFrontLeftDrive().setTargetPosition(LinearOpModeBase.COUNTS_PER_INCH * 60); // 68
        getBackRightDrive().setTargetPosition(-LinearOpModeBase.COUNTS_PER_INCH * 60); // 68

        getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        getBackRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run the other motors to drive at a steeper angle
        getFrontRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getBackLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getFrontLeftDrive().setPower(0.5);
        getBackRightDrive().setPower(0.5);

        // wait for the drive motors to stop
        while(opModeIsActive() &&
                (getFrontLeftDrive().isBusy() && getBackRightDrive().isBusy())) {

            // run the other motors to drive at a steeper angle
            getFrontRightDrive().setPower(0.12);
            getBackLeftDrive().setPower(-0.12);

            telemetry.addData("Path",  "Running at %d :%d",
                    getFrontLeftDrive().getCurrentPosition(),
                    getBackRightDrive().getCurrentPosition());

            telemetry.addData("front left target", getFrontLeftDrive().getTargetPosition());
            telemetry.addData("back right target", getBackRightDrive().getTargetPosition());
            telemetry.update();
            idle();
        }
        stopRobot();

        setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset before driving forward with touch sensor
        gyroPivot(0.8, 0, true);

        touchSensorDrive();

        // drive left to white line
        stopOnLine(0.1, RobotDirection.RIGHT);

        gyroPivot(0.8, 0, true);

        // claim the first beacon
        claimBeaconRed();

        // back up from wall
        rangeSensorDrive(20, 0.2);

        rangeGyroStrafe(0, 20, 35, RobotDirection.RIGHT);

//        rangeSensorDrive(10, 0.2);

        // reset before driving forward with touch sensor
        gyroPivot(0.8, 0, true);

        touchSensorDrive();

        // drive right to white line
        stopOnLine(0.1, RobotDirection.RIGHT);

        gyroPivot(0.8, 0, true);

        // claim the second beacon
        claimBeaconRed();

        // back up for launching
        rangeSensorDrive(20, 0.2);

        // gyro pivot for shooting
        gyroPivot(0.8, 46, true);

        // drive backward for shooting
        encoderDrive(0.5, 10, RobotDirection.BACKWARD);

        // open intake door
        getDoor3().setPosition(0.25);

        // launch the particle
        autoLaunchParticle();

        // pivot to eighty degrees
        gyroPivot(0.8, -85, true);

        encoderDriveDiagonal(0.5, 42, RobotDirection.NORTH_WEST);

        // drive left a foot
        encoderDrive(1.0, 12, RobotDirection.LEFT);
    }
}

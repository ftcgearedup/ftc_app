package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.TimeDriveAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.EncoderPivotAlgorithm;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.VuMarkScanAlgorithm;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.WiggleDriveAlgorithm;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;


/**
 * The Relic Recovery back blue alliance program.
 */
@Autonomous(name = "Blue Back", group = "autonomous")
public class AutonomousBlueBack extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    private VuMarkScanAlgorithm vuMarkScanAlgorithm;

    private DistanceSensorDriveAlgorithm rightDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm leftDistanceSensorDrive;

    private WiggleDriveAlgorithm wiggleDriveAlgorithm;

    private TimeDriveAlgorithm timeDriveAlgorithm;

    private EncoderPivotAlgorithm encoderPivotAlgorithm;

    private ElapsedTime timer;

    private double vuMarkScanTimeMS;

    private JSONConfigOptions configOptions;

    private int GLYPH_COLOR_SENSOR_THRESHOLD;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);

        this.configOptions = robot.getOptionsMap();

        GLYPH_COLOR_SENSOR_THRESHOLD = configOptions.retrieveAsInt("gcsThreshold");

        vuMarkScanTimeMS = robot.getOptionsMap().retrieveAsDouble("autonomousVuMarkScanTimeMS");

        // initialize vuforia
        robot.getVisionHelper().initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);

        vuMarkScanAlgorithm = new VuMarkScanAlgorithm(robot, robot.getVisionHelper());
        bno055IMUWrapper = new BNO055IMUWrapper(robot);
        gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);

        this.timeDriveAlgorithm = new TimeDriveAlgorithm(robot, robot.getHDriveTrain());
        this.wiggleDriveAlgorithm = new WiggleDriveAlgorithm(robot, robot.getHDriveTrain());
        this.encoderPivotAlgorithm = new EncoderPivotAlgorithm(robot, robot.getHDriveTrain());

        bno055IMUWrapper.startIntegration();

        rightDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getRightRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.RIGHT);

        leftDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getLeftRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.LEFT);

        this.timer = new ElapsedTime();

        robot.getGlyphLift().setIntakeInitializePosition();

        waitForStart();

        RelicRecoveryVuMark scannedVuMark = RelicRecoveryVuMark.UNKNOWN;

        vuMarkScanAlgorithm.activate();

        // set intake position to half open
        robot.getGlyphLift().setIntakeHalfOpenPosition();

        timer.reset();

        // scan VuMark
        while(opModeIsActive()
                && timer.milliseconds() < vuMarkScanTimeMS
                && scannedVuMark == RelicRecoveryVuMark.UNKNOWN) {
            scannedVuMark = vuMarkScanAlgorithm.detect();
        }

        vuMarkScanAlgorithm.deactivate();

        telemetry.addData("VuMark", scannedVuMark);
        telemetry.update();

        // knock jewel
        robot.getJewelKnocker().knockJewel(false);

        // move back to left position
        robot.getJewelKnocker().leftRotation();

        // drive off balancing stone to left
        robot.getHDriveTrain().directionalDrive(0, 1.0, 6, false);

        // drive forward to pit
        robot.getHDriveTrain().directionalDrive(90, 0.6, 24, false);

        // pivot to face glyph pit
        gyroPivotAlgorithm.pivot(1.0, 300, false, false);

        // drive forward a little more
        robot.getHDriveTrain().directionalDrive(90,0.5,12,false);

        // turn on intake and set extra intake to grip position
        robot.getGlyphLift().setGlyphIntakeMotorPower(-1);
        robot.getGlyphLift().setIntakeGripPosition();

        // reset timer
        timer.reset();

        // drive forward into glyph pile until
        while(opModeIsActive() && robot.getGlyphLift().getColorSensor().red() < GLYPH_COLOR_SENSOR_THRESHOLD && timer.milliseconds() < 2000) {
            robot.getHDriveTrain().drive(0, 0.5);
        }

        // stop driving
        robot.getHDriveTrain().stopDriveMotors();

        // drive out of pit
        robot.getHDriveTrain().directionalDrive(270, 0.5, 32, false);

        // face cryptobox in pile
        gyroPivotAlgorithm.pivot(0.5, 90, false, false);

        // drive forward in front of cryptobox
        robot.getHDriveTrain().directionalDrive(90, 0.5, 24, false);

        // align to middle column
        do {
            gyroPivotAlgorithm.pivot(0.1, 90, true, true);
            leftDistanceSensorDrive.driveToDistance(23, 1.0, true);
        } while(opModeIsActive() && leftDistanceSensorDrive.isAlgorithmBusy());

        switch (scannedVuMark) {
            case LEFT:
                gyroPivotAlgorithm.pivot(0.5, 113, false, false);
                break;
            case RIGHT:
                gyroPivotAlgorithm.pivot(0.5, 67, false, false);
                break;
            case UNKNOWN:
            case CENTER:
                break;
        }

        // drive forward into cryptobox
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 400) {
            robot.getHDriveTrain().drive(0, 1.0);
        }

        // run intake in reverse to eject glyph
        robot.getGlyphLift().ejectGlyph();

        // back up while ejecting glyph
        robot.getHDriveTrain().directionalDrive(270, 0.5, 6, false);

//        // ensure lift is stopped
//        robot.getGlyphLift().setLiftMotorPower(0);
//
//        robot.getHDriveTrain().drive(0.0, 0.0);
//
//        // back up while ejecting glyph
//        robot.getHDriveTrain().directionalDrive(270, 0.5, 6, false);
//
//        // don't gyro pivot after when key column is center
//        if(scannedVuMark == RelicRecoveryVuMark.LEFT || scannedVuMark == RelicRecoveryVuMark.RIGHT) {
//            // gyro pivot back to 270 after backing up
//            gyroPivotAlgorithm.pivot(0.5, 90, true, false);
//        }
//
//        // drive right to align with glyph pit
//        robot.getHDriveTrain().directionalDrive(180, 1.0, 16, false);
//
//        // turn to face glyph pit
//        encoderPivotAlgorithm.encoderPivot(0.5, 1400);
//
//        //gyroPivotAlgorithm.pivot(0.5, 90, false, false);
//        // timeDriveAlgorithm.pivot(0.8, 900);
//
//        // run intake
//        robot.getGlyphLift().setGlyphIntakeMotorPower(-1.0);
//        robot.getGlyphLift().setIntakeGripPosition();
//
//        // drive into glyph pit
//        robot.getHDriveTrain().directionalDrive(90, 1.0, 34, false);
//
//        // gyro pivot once in glyph pile
//        encoderPivotAlgorithm.encoderPivot(-0.5, 300);
////        gyroPivotAlgorithm.pivot(0.5, 45, false, false);
//
//        timer.reset();
//
//        // drive forward into glyph pile
//        while(opModeIsActive() && robot.getGlyphLift().getColorSensor().red() < GLYPH_COLOR_SENSOR_THRESHOLD && timer.milliseconds() < 3000) {
//
//        }
//
//        // stop after wiggle drive
//        robot.getHDriveTrain().stopDriveMotors();
//
//        // back up from glyph pit
//        robot.getHDriveTrain().directionalDrive(270, 1.0, 14, false);
//
//        // pivot to face cryptobox again
//        encoderPivotAlgorithm.encoderPivot(-0.5, 800);
//        //gyroPivotAlgorithm.pivot(0.5, 270, false, false);
//
//        // stop intake
//        robot.getGlyphLift().setGlyphIntakeMotorPower(0);
//        robot.getGlyphLift().setIntakeHalfOpenPosition();
//
//        timer.reset();
//
//        // drive right into balancing stone
//        while(opModeIsActive() && timer.milliseconds() < 2000) {
//            robot.getHDriveTrain().drive(-0.5, 0.0);
//            gyroPivotAlgorithm.pivot(0.3, 90, true, true);
//        }
//
//        // stop the robot
//        robot.getHDriveTrain().stopDriveMotors();
//
//        // drive left an inch off of balancing stone
//        robot.getHDriveTrain().directionalDrive(180, 1.0, 2, false);
//
//        // pivot after driving left off balancing stone
//        gyroPivotAlgorithm.pivot(0.5, 90, false, false);
//
//        // check if second glyph is on bottom and key column is right
//        if(robot.getGlyphLift().getColorSensor().red() > GLYPH_COLOR_SENSOR_THRESHOLD
//                && scannedVuMark == RelicRecoveryVuMark.RIGHT) {
//            robot.getGlyphLift().raiseGlyphLift();
//        }
//
//        // ensure robot is facing cryptobox
//        //gyroPivotAlgorithm.pivot(1.0, 270, false, false);
//
//        // drive forward to cryptobox
//        robot.getHDriveTrain().directionalDrive(90, 1.0, 24, false);
//
//        // wait between driving forward with encoders and time drive
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < 500) {
//            idle();
//        }
//
//        // time drive to push glyph in cryptobox
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < 500) {
//            robot.getHDriveTrain().drive(0, 0.7);
//        }
//
//        // run intake in reverse to eject glyph
//        robot.getGlyphLift().ejectGlyph();
//
//        // back up while ejecting glyph
//        robot.getHDriveTrain().directionalDrive(270, 0.5, 6, false);
//
//        // drive forward and back to ensure glyph is in
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < 500) {
//            robot.getHDriveTrain().drive(0, 0.7);
//        }
//
//        robot.getHDriveTrain().stopDriveMotors();
//
//        // wait half a second before reversing
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < 500) {
//            idle();
//        }
//
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < 500) {
//            robot.getHDriveTrain().drive(0, -1.0);
//        }
//
//        robot.getHDriveTrain().stopDriveMotors();
    }
}

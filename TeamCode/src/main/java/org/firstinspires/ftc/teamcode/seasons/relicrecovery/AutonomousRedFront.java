package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.EncoderPivotAlgorithm;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.VuMarkScanAlgorithm;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.WiggleDriveAlgorithm;
import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;


/**
 * The Relic Recovery front red alliance program.
 */
@Autonomous(name = "Red Front", group = "autonomous")
public class AutonomousRedFront extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    private VuMarkScanAlgorithm vuMarkScanAlgorithm;

    private DistanceSensorDriveAlgorithm rightDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm leftDistanceSensorDrive;

    private EncoderPivotAlgorithm encoderPivotAlgorithm;
    private WiggleDriveAlgorithm wiggleDriveAlgorithm;

    private ElapsedTime timer;

    private double vuMarkScanTimeMS;

    private JSONConfigOptions configOptions;

    private int GLYPH_COLOR_SENSOR_THRESHOLD;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);

        configOptions = robot.getOptionsMap();

        vuMarkScanTimeMS = configOptions.retrieveAsDouble("autonomousVuMarkScanTimeMS");
        GLYPH_COLOR_SENSOR_THRESHOLD = configOptions.retrieveAsInt("gcsThreshold");

        // initialize vuforia
        robot.getVisionHelper().initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);

        vuMarkScanAlgorithm = new VuMarkScanAlgorithm(robot, robot.getVisionHelper());
        bno055IMUWrapper = new BNO055IMUWrapper(robot);
        gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);

        this.encoderPivotAlgorithm = new EncoderPivotAlgorithm(robot, robot.getHDriveTrain());
        this.wiggleDriveAlgorithm = new WiggleDriveAlgorithm(robot, robot.getHDriveTrain());

        bno055IMUWrapper.startIntegration();

        rightDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getRightRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.RIGHT);

        leftDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getLeftRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.LEFT);

        this.timer = new ElapsedTime();

        waitForStart();

        RelicRecoveryVuMark scannedVuMark = RelicRecoveryVuMark.UNKNOWN;

        vuMarkScanAlgorithm.activate();

        // raise glyph lift and stop lift motors
        robot.getGlyphLift().raiseGlyphLift();

        timer.reset();

        // scan VuMark
        while (opModeIsActive()
                && timer.milliseconds() < vuMarkScanTimeMS
                && scannedVuMark == RelicRecoveryVuMark.UNKNOWN) {
            scannedVuMark = vuMarkScanAlgorithm.detect();

            // stop glyph lift if it is at its target position
            if (!robot.getGlyphLift().isGlyphLiftBusy()) {
                robot.getGlyphLift().setLiftMotorPower(0);
            }
        }

        vuMarkScanAlgorithm.deactivate();

        telemetry.addData("VuMark", scannedVuMark);
        telemetry.update();

        // knock jewel
        robot.getJewelKnocker().knockJewel(true);

        // move back to left position
        robot.getJewelKnocker().rightRotation();

        // drive off balancing stone
        robot.getHDriveTrain().directionalDrive(180, 1.0, 26, false);

        // drive forward a bit before pivoting to face the cryptobox
        robot.getHDriveTrain().directionalDrive(90, 0.5, 3, false);

        // pivot to face cryptobox
        encoderPivotAlgorithm.encoderPivot(0.5, 1400);
        //gyroPivotAlgorithm.pivot(0.5, 180, true, false);

        // lower the lift
        robot.getGlyphLift().setLiftMotorPower(-robot.getGlyphLift().MAX_LIFT_MOTOR_POWER_DOWN);

        // align to middle column
        do {
            // stop lift if the lift touch sensor is pressed
            if (robot.getGlyphLift().isLiftTouchSensorPressed()) {
                robot.getGlyphLift().setLiftMotorPower(0);
            }

            gyroPivotAlgorithm.pivot(0.1, 180, true, true);
            leftDistanceSensorDrive.driveToDistance(15.5, 1.0, true);
        } while (opModeIsActive() && leftDistanceSensorDrive.isAlgorithmBusy());

        switch (scannedVuMark) {
            case LEFT:
                gyroPivotAlgorithm.pivot(0.5, 203, false, false);
                break;
            case RIGHT:
                gyroPivotAlgorithm.pivot(0.5, 157, false, false);
                break;
            case UNKNOWN:
            case CENTER:
                break;
        }

        // drive forward into cryptobox
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 400) {
            robot.getHDriveTrain().drive(0, 1.0);
        }

        robot.getHDriveTrain().drive(0.0, 0.0);

        // run intake in reverse to eject glyph
        robot.getGlyphLift().ejectGlyph();

        // back up while ejecting glyph
        robot.getHDriveTrain().directionalDrive(270, 0.5, 10, false);

        // gyro pivot back to 0 after backing up to face glyph pit
        if(scannedVuMark == RelicRecoveryVuMark.LEFT || scannedVuMark == RelicRecoveryVuMark.RIGHT) {
            gyroPivotAlgorithm.pivot(0.5, 180, true, false);
        }

        // encoder pivot to face glyph pit
        encoderPivotAlgorithm.encoderPivot(0.5, 1400);

        // run intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(-1.0);

        // drive into glyph pit
        robot.getHDriveTrain().directionalDrive(90, 1.0, 24, false);

        timer.reset();

        // wiggle-drive forward into glyph pile
        while(opModeIsActive() && robot.getGlyphLift().getColorSensor().red() < GLYPH_COLOR_SENSOR_THRESHOLD && timer.milliseconds() < 3000) {
            wiggleDriveAlgorithm.drive(0.7, 500);
        }

        // back up out of glyph pit
        robot.getHDriveTrain().directionalDrive(270, 0.5, 24, false);

        // stop the intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(0);

        // pivot to face cryptobox
        gyroPivotAlgorithm.pivot(0.5, 180, false, false);

        timer.reset();

        // drive right into balancing stone
        while(opModeIsActive() && timer.milliseconds() < 2000) {
            robot.getHDriveTrain().drive(0.5, 0.0);
            gyroPivotAlgorithm.pivot(0.2, 180, true, true);
        }

        // align to middle column
        do {
            gyroPivotAlgorithm.pivot(0.1, 180, true, true);
            leftDistanceSensorDrive.driveToDistance(15.5, 1.0, true);
        } while (opModeIsActive() && leftDistanceSensorDrive.isAlgorithmBusy());

        boolean hasTwoGlyphs =
                robot.getGlyphLift().getColorSensor().red() > GLYPH_COLOR_SENSOR_THRESHOLD
                && robot.getGlyphLift().getGlyphTouchSensor().isPressed();

        if(!hasTwoGlyphs) {
            switch (scannedVuMark) {
                case LEFT:
                    gyroPivotAlgorithm.pivot(0.5, 203, false, false);
                    break;
                case RIGHT:
                    gyroPivotAlgorithm.pivot(0.5, 157, false, false);
                    break;
                case UNKNOWN:
                case CENTER:
                    break;
                }
        } else {
            switch (scannedVuMark) {
                case RIGHT:
                case LEFT:
                    break;
                case UNKNOWN:
                case CENTER:
                    // pivot to left
                    gyroPivotAlgorithm.pivot(0.5, 203, false, false);

            }
        }

//        DistanceSensorDriveAlgorithm driveAlgorithmToUse;
//        double targetDistance = 21.5;
//
//        // if the robot collected two glyphs
//        if(robot.getGlyphLift().getColorSensor().red() > GLYPH_COLOR_SENSOR_THRESHOLD
//                && robot.getGlyphLift().getGlyphTouchSensor().isPressed()) {
//            if (scannedVuMark == RelicRecoveryVuMark.RIGHT) {
//                // put in left
//                driveAlgorithmToUse = rightDistanceSensorDrive;
//            } else {
//                // put in right
//                driveAlgorithmToUse = leftDistanceSensorDrive;
//            }
//        } else {
//            if(scannedVuMark == RelicRecoveryVuMark.RIGHT) {
//                // put in right
//                driveAlgorithmToUse = leftDistanceSensorDrive;
//            } else if(scannedVuMark == RelicRecoveryVuMark.LEFT) {
//                // put in left
//                driveAlgorithmToUse = rightDistanceSensorDrive;
//            } else {
//                targetDistance = 15.5;
//
//                // always use distance sensor for the center column
//                if(robot.getLeftRangeSensor().getDistance(DistanceUnit.INCH) >
//                        robot.getRightRangeSensor().getDistance(DistanceUnit.INCH)) {
//                    driveAlgorithmToUse = leftDistanceSensorDrive;
//                } else {
//                    driveAlgorithmToUse = rightDistanceSensorDrive;
//                }
//            }
//        }
//
//        // align to column for second or third glyphs
//        do {
//            gyroPivotAlgorithm.pivot(0.1, 180, true, true);
//            driveAlgorithmToUse.driveToDistance(targetDistance, 1.0, true);
//        } while (opModeIsActive() && driveAlgorithmToUse.isAlgorithmBusy());

        // drive forward into cryptobox
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 750) {
            robot.getHDriveTrain().drive(0, 1.0);
        }

        robot.getHDriveTrain().drive(0.0, 0.0);

        // run intake in reverse to eject glyph
        robot.getGlyphLift().ejectGlyph();

        // back up while ejecting glyph
        robot.getHDriveTrain().directionalDrive(270, 0.5, 10, false);

        // drive forward and back to ensure glyph is in
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 500) {
            robot.getHDriveTrain().drive(0, 1.0);
        }

        robot.getHDriveTrain().stopDriveMotors();

        // wait half a second before reversing
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 500) {
            idle();
        }

        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 250) {
            robot.getHDriveTrain().drive(0, -1.0);
        }

        robot.getHDriveTrain().stopDriveMotors();
    }
}

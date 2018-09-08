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
 * The Relic Recovery back red alliance program.
 */
@Autonomous(name = "Red Back \uD83D\uDE08", group = "autonomous")
public class AutonomousRedBack extends LinearOpMode {

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
    private double GYRO_PIVOT_SPEED;
    private double MAX_RANGE_DRIVE_DISTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);

        this.configOptions = robot.getOptionsMap();

        GLYPH_COLOR_SENSOR_THRESHOLD = configOptions.retrieveAsInt("gcsThreshold");
        GYRO_PIVOT_SPEED = robot.getOptionsMap().retrieveAsDouble("autonomousGyroPivotSpeed");
        MAX_RANGE_DRIVE_DISTANCE = robot.getOptionsMap().retrieveAsDouble("maxRangeDriveDistance");

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

        // set intake position to half open
        robot.getGlyphLift().setIntakeHalfOpenPosition();

        vuMarkScanAlgorithm.activate();

        // knock jewel
        robot.getJewelKnocker().knockJewel(true);

        // move back to left position
        robot.getJewelKnocker().leftRotation();

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

        // drive off balancing stone to right
        robot.getHDriveTrain().directionalDrive(180, 1.0, 6, false);

        // drive forward to pit
        robot.getHDriveTrain().directionalDrive(90, 0.6, 24, false);

        // pivot to face glyph pit
        encoderPivotAlgorithm.encoderPivot(-0.5, 400);

        // drive forward a little more
        robot.getHDriveTrain().directionalDrive(90,0.5,14,false);

        // turn on intake and set extra intake to grip position
        robot.getGlyphLift().setGlyphIntakeMotorPower(-1);
        robot.getGlyphLift().setIntakeGripPosition();

        // reset timer
        timer.reset();

        // drive forward into glyph pile until
        while(opModeIsActive() && robot.getGlyphLift().getColorSensor().red() < GLYPH_COLOR_SENSOR_THRESHOLD && timer.milliseconds() < 2000) {
            wiggleDriveAlgorithm.drive(1.0,250);
        }

        // stop driving
        robot.getHDriveTrain().stopDriveMotors();

        // drive out of pit
        robot.getHDriveTrain().directionalDrive(270, 0.5, 15, false);

        // face cryptobox
        gyroPivotAlgorithm.pivot(GYRO_PIVOT_SPEED, 270, false, false);

        // turn off intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(0);
        robot.getGlyphLift().setIntakeHalfOpenPosition();

        // drive forward in front of cryptobox
        robot.getHDriveTrain().directionalDrive(90, 0.5, 20, false);

        // align to key column
        do {
            double columnDist = 26.5;
            switch (scannedVuMark) {
                case LEFT:
//                    gyroPivotAlgorithm.pivot(0.5, 113, false, false);
                    columnDist = 32;
                    break;
                case RIGHT:
                    //  gyroPivotAlgorithm.pivot(0.5, 67, false, false);
                    columnDist = 21;
                    break;
                case UNKNOWN:
                case CENTER:
                    break;
            }
            gyroPivotAlgorithm.pivot(0.1, 270, true, true);
            rightDistanceSensorDrive.driveToDistance(columnDist, MAX_RANGE_DRIVE_DISTANCE, 1.0, true);
        } while(opModeIsActive() && rightDistanceSensorDrive.isAlgorithmBusy());

        // open intake so it won't interfere with turning the glyph
        robot.getGlyphLift().setIntakeHalfOpenPosition();

        // turn glyphs to a 45 degree angle
        robot.getGlyphLift().spinWheelsInDirection(true,1.0);

        // turn off intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(0);

        // drive forward into cryptobox
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 600) {
            robot.getHDriveTrain().drive(0, 0.7);
        }

        // run intake in reverse to eject glyph
        robot.getGlyphLift().ejectGlyph();

        // back up while ejecting glyph
        robot.getHDriveTrain().directionalDrive(270, 1.0, 8, false);

        // ensure lift is stopped
        robot.getGlyphLift().setLiftMotorPower(0);

        // stop drive motors
        robot.getHDriveTrain().stopDriveMotors();

        // turn off intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(0);

        if(scannedVuMark != RelicRecoveryVuMark.LEFT) {
            // drive right to align with glyph pit
            do {
                rightDistanceSensorDrive.driveToDistance(32, MAX_RANGE_DRIVE_DISTANCE, 1, false);
            } while (opModeIsActive() && rightDistanceSensorDrive.isAlgorithmBusy());
        }

        // turn to face glyph pit
        gyroPivotAlgorithm.pivot(GYRO_PIVOT_SPEED, 65, true, false);

        // run intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(-1.0);
        robot.getGlyphLift().setIntakeGripPosition();

        // drive into glyph pit
        robot.getHDriveTrain().directionalDrive(90, 1.0, 34, false);

        timer.reset();

        // drive forward into glyph pile
        while(opModeIsActive() && robot.getGlyphLift().getColorSensor().red() < GLYPH_COLOR_SENSOR_THRESHOLD && timer.milliseconds() < 3000) {
            wiggleDriveAlgorithm.drive(1.0,250);
        }

        // stop after drive
        robot.getHDriveTrain().stopDriveMotors();

        robot.getHDriveTrain().directionalDrive(270, 0.5, 44, false);

        // turn off intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(0);
        robot.getGlyphLift().setIntakeHalfOpenPosition();
    }
}

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
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.VuMarkScanAlgorithm;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.WiggleDriveAlgorithm;


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

    private ElapsedTime timer;

    private double vuMarkScanTimeMS;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);

        vuMarkScanTimeMS = robot.getOptionsMap().retrieveAsDouble("autonomousVuMarkScanTimeMS");

        // initialize vuforia
        robot.getVisionHelper().initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);

        vuMarkScanAlgorithm = new VuMarkScanAlgorithm(robot, robot.getVisionHelper());
        bno055IMUWrapper = new BNO055IMUWrapper(robot);
        gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);

        this.timeDriveAlgorithm = new TimeDriveAlgorithm(robot, robot.getHDriveTrain());
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
        while(opModeIsActive()
                && timer.milliseconds() < vuMarkScanTimeMS
                && scannedVuMark == RelicRecoveryVuMark.UNKNOWN) {
            scannedVuMark = vuMarkScanAlgorithm.detect();

            // stop glyph lift if it is at its target position
            if(!robot.getGlyphLift().isGlyphLiftBusy()) {
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
        robot.getHDriveTrain().directionalDrive(0, 1.0, 24, false);

        // pivot to face cryptobox
        // gyroPivotAlgorithm.pivot(0.5, 270, true, false);
        timeDriveAlgorithm.pivot(1.0, 350);

        // lower the lift
        robot.getGlyphLift().setLiftMotorPower(-robot.getGlyphLift().MAX_LIFT_MOTOR_POWER_DOWN);

        // align to middle column
        do {
            // stop lift if the lift touch sensor is pressed
            if(robot.getGlyphLift().isLiftTouchSensorPressed()) {
                robot.getGlyphLift().setLiftMotorPower(0);
            }

            gyroPivotAlgorithm.pivot(0.1, 270, true, true);
            leftDistanceSensorDrive.driveToDistance(25, 1.0, true);
        } while(opModeIsActive() && leftDistanceSensorDrive.isAlgorithmBusy());

        switch (scannedVuMark) {
            case LEFT:
                gyroPivotAlgorithm.pivot(0.5, 293, false, false);
                break;
            case RIGHT:
                gyroPivotAlgorithm.pivot(0.5, 247, false, false);
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

        robot.getHDriveTrain().drive(0.0, 0.0);

        // run intake in reverse to eject glyph
        robot.getGlyphLift().ejectGlyph();

        // back up while ejecting glyph
        robot.getHDriveTrain().directionalDrive(270, 0.5, 6, false);

        // gyro pivot back to 270 after backing up
        gyroPivotAlgorithm.pivot(0.5, 270, true, false);

        // drive left to align with glyph pit
        robot.getHDriveTrain().directionalDrive(0, 1.0, 20, false);

        // turn to face glyph pit
        //gyroPivotAlgorithm.pivot(0.5, 90, false, false);
        timeDriveAlgorithm.pivot(1.0, 750);

        // run intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(-1.0);

        // drive into glyph pit
        robot.getHDriveTrain().directionalDrive(90, 1.0, 30, false);

        // gyro pivot once in glyph pile
        gyroPivotAlgorithm.pivot(0.5, 45, false, false);

        // wiggle-drive forward into glyph pile
        wiggleDriveAlgorithm.drive(1.0, 500, 3000);

        // stop intake
        robot.getGlyphLift().setGlyphIntakeMotorPower(0);

    }
}

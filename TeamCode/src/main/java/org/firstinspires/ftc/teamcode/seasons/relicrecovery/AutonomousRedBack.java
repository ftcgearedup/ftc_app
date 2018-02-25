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
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.VuMarkScanAlgorithm;


/**
 * Created by Owner on 12/5/2017.
 */
@Autonomous(name = "Red Back", group = "autonomous")
public class AutonomousRedBack extends LinearOpMode {

    private RelicRecoveryRobot robot;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    private VuMarkScanAlgorithm vuMarkScanAlgorithm;

    private DistanceSensorDriveAlgorithm frontDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm rightDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm leftDistanceSensorDrive;

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

        bno055IMUWrapper.startIntegration();

        frontDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getFrontRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.FRONT);

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

        robot.getGlyphLift().raiseGlyphLift();

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
        robot.getJewelKnocker().knockJewel(true);

        // drive off balancing stone
        robot.getHDriveTrain().directionalDrive(180, 0.5, 24, false);

        // pivot to face cryptobox
        gyroPivotAlgorithm.pivot(0.5, 270, true, false);

        // lower the lift
        robot.getGlyphLift().lowerGlyphLift();

        // align to middle column
        do {
            gyroPivotAlgorithm.pivot(0.1, 270, true, true);
            rightDistanceSensorDrive.driveToDistance(25, 1.0, true);
        } while(rightDistanceSensorDrive.isAlgorithmBusy());

        switch (scannedVuMark) {
            case UNKNOWN:
                robot.getHDriveTrain().directionalDrive(90, 0.5, 6,false);
                break;
        }

        robot.getGlyphLift().ejectGlyph();
    }
}

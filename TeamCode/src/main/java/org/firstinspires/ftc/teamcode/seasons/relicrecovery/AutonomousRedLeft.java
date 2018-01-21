package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.mechanism.impl.VisionHelper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.VuMarkScanAlgorithm;


/**
 * Created by Owner on 12/5/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Left", group = "autonomous")
public class AutonomousRedLeft extends LinearOpMode {

    private RelicRecoveryRobot robot;
    private VuMarkScanAlgorithm vuMarkScanAlgorithm;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;
    private DistanceSensorDriveAlgorithm frontDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm rightDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm leftDistanceSensorDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RelicRecoveryRobot(this);
        vuMarkScanAlgorithm = new VuMarkScanAlgorithm(robot, robot.getVisionHelper());
        bno055IMUWrapper = new BNO055IMUWrapper(robot);
        gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);

        bno055IMUWrapper.startIntegration();

        frontDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getFrontRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.FRONT);

        rightDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getRightRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.FRONT);

        leftDistanceSensorDrive = new DistanceSensorDriveAlgorithm(
                robot, robot.getHDriveTrain(), robot.getLeftRangeSensor(),
                DistanceSensorDriveAlgorithm.RobotSide.FRONT);

        //detect color of stone
        boolean isStoneRed = true;
        boolean isStoneRight = false;


        ElapsedTime driveTimer = new ElapsedTime();

//        if(robot.balancingStoneSensor.red() > 0){
//            isStoneRed = true;
//            telemetry.addData(">", "Red stone detected.");
//        } else {
//            isStoneRed = false;
//            telemetry.addData(">", "Blue stone detected.");
//        }

        robot.getGlyphLift().initializeGrippers();
        robot.getIntake().raiseIntake();
        robot.getJewelKnocker().retractArm();

        waitForStart();

        RelicRecoveryVuMark scannedVuMark = RelicRecoveryVuMark.CENTER;

        vuMarkScanAlgorithm.activate();

        driveTimer.reset();
        while (driveTimer.milliseconds() < 1000) {
            scannedVuMark = vuMarkScanAlgorithm.detect();
        }

        if(scannedVuMark == RelicRecoveryVuMark.UNKNOWN) {
            scannedVuMark = RelicRecoveryVuMark.CENTER;
        }

        vuMarkScanAlgorithm.deactivate();

        telemetry.addData("VuMark", scannedVuMark);
        telemetry.update();

        robot.getGlyphLift().closeRedGripper();

        sleep(500);

        robot.getGlyphLift().setLiftMotorPower(1);
        sleep(250);
        robot.getGlyphLift().setLiftMotorPower(0);

        // Lower Jewel Mechinism
        robot.getJewelKnocker().extendArm();

        sleep(1000);

        if (robot.getJewelKnocker().isJewelRed()) {
            telemetry.addData(">", "jewel is red");
            telemetry.update();

            robot.getHDriveTrain().directionalDrive(180, 0.5, 2, false); //drive 4 inches right

            robot.getJewelKnocker().retractArm();

            robot.getHDriveTrain().directionalDrive(180, 1.0, 18, false); //drive 4 inches right
            sleep(500);
        } else if (robot.getJewelKnocker().isJewelBlue()) {
            telemetry.addData(">", "jewel is blue");
            telemetry.update();

            robot.getHDriveTrain().directionalDrive(0, 0.5, 2, false); // drive 4 inches left

            robot.getJewelKnocker().retractArm();

            robot.getHDriveTrain().directionalDrive(180, 1.0, 22, false); //drive 4 inches right
        }

        // gyro pivot to zero degree angle
        gyroPivotAlgorithm.pivot(0.5, 0, true, false);

        // drive forward before driving into balancing stone
        robot.getHDriveTrain().directionalDrive(270, 0.3, 4, false); //drive 4 inches right

        driveTimer.reset();

        // drive of balancing stone
        while(driveTimer.milliseconds() < 1000) {
            robot.getHDriveTrain().drive(0.5, 0.0);
        }
        robot.getHDriveTrain().stopDriveMotors();

////        // pivot to face cryptobox
////        gyroPivotAlgorithm.pivot(0.5, 180, true, false);
//
//        robot.getHDriveTrain().directionalDrive(180, 0.5, 24, false);
//
//        robot.getHDriveTrain().directionalDrive(0, 0.5, 4, false);

        // drive right/left to face key column
        switch (scannedVuMark) {
            case UNKNOWN:
            case CENTER:
                robot.getHDriveTrain().directionalDrive(180, 0.5, 16, false);
//                rightDistanceSensorDrive.driveToDistance(52, 0.5, false);
                break;
            case LEFT:
                robot.getHDriveTrain().directionalDrive(180, 0.5, 23, false);
                break;
            case RIGHT:
                robot.getHDriveTrain().directionalDrive(180, 0.5, 9, false);
                break;
        }

        // pivot to face cryptobox
        gyroPivotAlgorithm.pivot(0.3, 180, true, false);

        robot.getGlyphLift().setLiftMotorPower(-0.2);
        sleep(750);
        robot.getGlyphLift().setLiftMotorPower(0.2);

        driveTimer.reset();

        // drive into cryptobox
        while(driveTimer.milliseconds() < 1000) {
            robot.getHDriveTrain().drive(0, -0.5);
        }
        robot.getHDriveTrain().stopDriveMotors();

        robot.getGlyphLift().openRedGripper();
        robot.getHDriveTrain().directionalDrive(90, 0.5, 4, false);

        driveTimer.reset();

        // drive into cryptobox
        while(driveTimer.milliseconds() < 1000) {
            robot.getHDriveTrain().drive(0, -0.5);
        }
        robot.getHDriveTrain().stopDriveMotors();

        robot.getHDriveTrain().directionalDrive(90, 0.5, 12, false);
    }
}

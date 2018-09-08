package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;

@TeleOp(name = "Gyro Pivot Test", group = "testing")
public class GyroPivotTest extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private BNO055IMUGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RelicRecoveryRobot(this);
        this.bno055IMUWrapper = new BNO055IMUWrapper(robot);
        this.gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getHDriveTrain(), bno055IMUWrapper);
        this.timer = new ElapsedTime();

        bno055IMUWrapper.startIntegration();

        double speed = robot.getOptionsMap().retrieveAsDouble("autonomousGyroPivotSpeed");

        while(!isStarted() && !opModeIsActive()) {
            telemetry.addData("heading", bno055IMUWrapper.getHeading());
            telemetry.addData("error", gyroPivotAlgorithm.getError(180, true));
            telemetry.update();
        }

        gyroPivotAlgorithm.pivot(speed, 180, true, false);

        timer.reset();
        while(timer.seconds() < 300 && opModeIsActive()){
            telemetry.addData("heading", bno055IMUWrapper.getHeading());
            telemetry.addData("error", gyroPivotAlgorithm.getError(180, true));
            telemetry.addData("Threshold", robot.getOptionsMap().retrieveAsDouble("gyroPivotGyroDegreeThreshold"));
            telemetry.addData("P Coefficient", robot.getOptionsMap().retrieveAsDouble("gyroPivotPCoeff"));
            telemetry.addData("I Coefficient", robot.getOptionsMap().retrieveAsDouble("gyroPivotICoeff"));
            telemetry.addData("D Coefficient", robot.getOptionsMap().retrieveAsDouble("gyroPivotDCoeff"));
            telemetry.update();
            idle();
        }
    }
}

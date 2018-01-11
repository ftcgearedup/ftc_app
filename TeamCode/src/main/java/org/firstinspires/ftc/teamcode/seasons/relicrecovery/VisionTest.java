package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;

/**
 * Created by ftc6347 on 12/19/17.
 */
@Autonomous(name = "Vision Test")
public class VisionTest extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RelicRecoveryRobot(this);
        this.bno055IMUWrapper = new BNO055IMUWrapper(robot);

        this.gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(
                robot, robot.getHDriveTrain(), bno055IMUWrapper);

        waitForStart();

        gyroPivotAlgorithm.pivot(0.5, 90, false, true);

        //Mat frame = robot.getVisionHelper().readOpenCVFrame();
        //Highgui.imwrite(AppUtil.ROBOT_DATA_DIR + "capture.jpg", frame);

        //telemetry.addData(">", "Captured frame!");
        //telemetry.update();
    }
}

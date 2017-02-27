package org.firstinspires.ftc.teamcode.mentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by aburger on 2/27/2017.
 */
//@Disabled
@TeleOp(name = "Programming Robot Test", group = "Mentor")
public class TestProgrammingRobot extends LinearOpMode {
    private ProgrammingRobotHardwareV2 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = ProgrammingRobotHardwareV2.getInstance();
        robot.init(hardwareMap, telemetry);

        robot.getGyroSensor().calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.getGyroSensor().isCalibrating()) {
            sleep(50);
            telemetry.addData(">", "Calibrating Gyro");
            telemetry.update();
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData(">", "Integrated Z value = %d", robot.getGyroSensor().getIntegratedZValue());
            telemetry.update();
            idle();
        }

        while (opModeIsActive()) {
            telemetry.addData("light sensor", robot.getFrontLightSensor().getRawLightDetected());
            telemetry.update();
        }

    }
}

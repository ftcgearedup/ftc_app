package org.firstinspires.ftc.teamcode.teaching;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.BNO055IMUGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.TimeDriveAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;

import java.sql.Time;

@Autonomous(name = "DemoAuto", group = "Autonomous")
public class DemoAuto extends LinearOpMode {

    private DemoRobot robot;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    private TimeDriveAlgorithm timeDriveAlgorithm;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DemoRobot(this,"testOptions.json");

        bno055IMUWrapper = new BNO055IMUWrapper(robot);
        gyroPivotAlgorithm = new BNO055IMUGyroPivotAlgorithm(robot, robot.getDemoDriveTrain(), bno055IMUWrapper);
        bno055IMUWrapper.startIntegration();

        timeDriveAlgorithm = new TimeDriveAlgorithm(robot, robot.getDemoDriveTrain());

        waitForStart();
        telemetry.addData("pivoting","");
        telemetry.update();

//        telemetry.addData("testSpeed",robot.getOptionsMap().retrieveAsDouble("testSpeed"));
//        telemetry.addData("PCoeff",robot.getOptionsMap().retrieveAsDouble("gyroPivotPCoeff"));
//
//        telemetry.update();
        gyroPivotAlgorithm.pivot(robot.getOptionsMap().retrieveAsDouble("testSpeed"),
                180,true,false);

        while (opModeIsActive()) {

        break;
        }
    }
}
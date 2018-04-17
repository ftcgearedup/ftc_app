package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.VuMarkScanAlgorithm;

@Autonomous(name = "JewelTest")
public class JewelTest extends LinearOpMode {
    private RelicRecoveryRobot robot;
    private VuMarkScanAlgorithm vuMarkScanAlgorithm;
    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RelicRecoveryRobot(this);
        // initialize vuforia
        robot.getVisionHelper().initializeVuforia(VuforiaLocalizer.CameraDirection.BACK);

        vuMarkScanAlgorithm = new VuMarkScanAlgorithm(robot, robot.getVisionHelper());

        waitForStart();

        RelicRecoveryVuMark scannedVuMark = RelicRecoveryVuMark.UNKNOWN;

        vuMarkScanAlgorithm.activate();

        telemetry.addData("Jewel", "Turned On");
        telemetry.update();

        robot.getJewelKnocker().knockJewel(true);

        telemetry.addData("Scan Vumark", "Turned On");
        telemetry.update();

        // scan VuMark
        timer.reset();
        while(opModeIsActive()
                && timer.milliseconds() < 2000
                && scannedVuMark == RelicRecoveryVuMark.UNKNOWN) {
            scannedVuMark = vuMarkScanAlgorithm.detect();
        }
        telemetry.addData("VuMark", scannedVuMark.toString());
        telemetry.update();
    }
}

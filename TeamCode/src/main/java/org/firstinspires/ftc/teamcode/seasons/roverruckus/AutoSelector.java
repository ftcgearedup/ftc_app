package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.VufTFLiteHandler;
@Autonomous(name = "AutoSelector", group = "Autonomous")
public class AutoSelector extends VufTFLiteHandler {
    public ElapsedTime land = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
    @Override
    public void runOpMode() throws InterruptedException {
        while (gamepad1.left_trigger != 1 && !isStarted()) {
            if (gamepad1.dpad_down) {
                telemetry.addData("program- ", "lander crater");
                telemetry.update();
                land.reset();
                while (land.milliseconds() <= 500) {
                    telemetry.update();
                }
                while (gamepad1.left_trigger != 1 && !isStarted()) {
                    if (gamepad1.dpad_down) {
                        telemetry.clear();
                        telemetry.addData("program- ", "ground crater");
                        telemetry.update();
                    } else if (gamepad1.dpad_right) {
                        break;
                    }
                }
            } else if (gamepad1.dpad_up && !isStarted()) {
                telemetry.addData("program- ", "lander depot");
                telemetry.update();
                land.reset();
                while (land.milliseconds() <= 500) {
                    telemetry.update();
                }
                while (gamepad1.left_trigger != 1 && !isStarted()) {
                    if (gamepad1.dpad_up) {
                        telemetry.clear();
                        telemetry.addData("program- ", "ground depot");
                        telemetry.update();
                    } else if (gamepad1.dpad_right) {
                        break;
                    }
                }
            }
        }
        waitForStart();
    }
}

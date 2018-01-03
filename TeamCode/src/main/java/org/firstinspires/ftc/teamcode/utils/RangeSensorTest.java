package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by daniel on 12/29/17.
 */
@TeleOp(name = "Range Sensor Test", group = "tests")
public class RangeSensorTest extends LinearOpMode {

    private AnalogInput sensorAnalogInput;

    private static double MILLIVOLTS_PER_INCH = 3.1;

    @Override
    public void runOpMode() throws InterruptedException {
        this.sensorAnalogInput = hardwareMap.analogInput.get("rangeSensor");

        waitForStart();

        double milliVolts;
        double distanceInches;

        while(opModeIsActive()) {
            milliVolts = sensorAnalogInput.getVoltage() * 1000;
            distanceInches = milliVolts / MILLIVOLTS_PER_INCH;

            telemetry.addData("volts", sensorAnalogInput.getVoltage());
            telemetry.addData("distance inches", "%.2f", distanceInches);
            telemetry.update();
        }

    }
}

package org.firstinspires.ftc.teamcode.mechanism.impl;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * {@link MaxSonarEZ4Sensor} implements the {@link DistanceSensor}
 * interface for the MB1040 LV-MaxSonar-EZ4 rangefinder.
 * <p>
 * This class requires an analog device to be defined in the hardware map
 * as this implementation uses analog to communicate with the sensor.
 *
 * @see <a href="https://www.maxbotix.com/Ultrasonic_Sensors/MB1040.htm">MB1040 sensor documentation</a>
 */

public class MaxSonarEZ4Sensor implements DistanceSensor {

    private AnalogInput analogInput;

    private static double MILLIVOLTS_PER_INCH = 3.1;

    /**
     * Creates a new MaxSonarEZ4Sensor instance.
     *
     * @param robot the robot
     * @param hardwareMapName the name of the sensor in the hardware
     */
    public MaxSonarEZ4Sensor(Robot robot, String hardwareMapName) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;

        // get analog input device that will be used for reading values from the sensor
        this.analogInput = hwMap.analogInput.get(hardwareMapName);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        // multiply by 1000 to convert to milli-volts
        double milliVolts = analogInput.getVoltage() * 1000;

        // divide by the milli-volt difference per inch and round to the nearest tenth
        return Math.round(unit.fromInches(milliVolts / MILLIVOLTS_PER_INCH) * 10) / 10;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "LV MaxSonar-EZ4";
    }

    @Override
    public String getConnectionInfo() {
        return analogInput.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        analogInput.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        analogInput.close();
    }
}

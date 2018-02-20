package org.firstinspires.ftc.teamcode.mechanism.impl;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * {@link MaxSonarEZ4AbstractSensor} is an abstract class that implements the {@link DistanceSensor}
 * interface for the MaxBotix MaxSonar-EZ4 range sensor series. Each sensor model in this series has
 * different distance units and minimum multiples of that unit. Extending classes should call
 * the constructor and pass the appropriate arguments for these values.
 * <p>
 * This class requires an analog device to be defined in the hardware map
 * as this implementation uses analog to communicate with the sensor.
 *
 * @see <a href="https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf">MaxSonar EZ datasheet</a>
 */

public abstract class MaxSonarEZ4AbstractSensor implements DistanceSensor {

    private DistanceUnit fromUnit;
    private double unitMultiple;

    private AnalogInput analogInput;

    private static final double SENSOR_VOLTAGE = 3.3;
    private static final double CONVERSION_FACTOR = SENSOR_VOLTAGE / 1024;
    private static final double MILLIVOLTS_PER_UNIT = CONVERSION_FACTOR * 1000;

    /**
     * Creates a new MaxSonarEZ4AbstractSensor instance.
     *
     * @param robot the robot
     * @param hardwareMapName the name of the sensor in the hardware
     * @param unit the unit in which this sensor model reads data
     * @param unitMultiple the minimum multiple of the unit this sensor reads
     */
    public MaxSonarEZ4AbstractSensor(Robot robot, String hardwareMapName,
                                     DistanceUnit unit, double unitMultiple) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;

        this.fromUnit = unit;
        this.unitMultiple = unitMultiple;

        // get analog input device that will be used for reading values from the sensor
        this.analogInput = hwMap.analogInput.get(hardwareMapName);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        // multiply by 1000 to convert to milli-volts
        double milliVolts = analogInput.getVoltage() * 1000;

        // divide by the milli-volt difference per inch and round to the nearest tenth
        return unit.fromUnit(fromUnit, milliVolts / MILLIVOLTS_PER_UNIT) * unitMultiple;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MaxSonar-EZ4";
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

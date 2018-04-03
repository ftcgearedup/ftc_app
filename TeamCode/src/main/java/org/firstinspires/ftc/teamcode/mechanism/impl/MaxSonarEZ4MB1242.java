package org.firstinspires.ftc.teamcode.mechanism.impl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Wire;

public class MaxSonarEZ4MB1242 implements DistanceSensor {

    private Wire sensorWire;
    private OpMode opMode;

    private double lastRangeReading = 0;

    private static final int ADDRESS_I2C_DEFAULT = 0x70;
    private static final int RANGE_READING_COMMAND_DELAY_MS = 100;

    /**
     * Construct a new {@link MaxSonarEZ4MB1242} instance.
     *
     * @param robot the robot using this sensor
     * @param hardwareMapName the name of the i2c device that represents
     *                        this sensor in the configuration file
     */
    public MaxSonarEZ4MB1242(Robot robot, String hardwareMapName) {
        this.opMode = robot.getCurrentOpMode();
        sensorWire = new Wire(opMode.hardwareMap, hardwareMapName, ADDRESS_I2C_DEFAULT);

        takeRangeReading();
    }

    private void takeRangeReading() {
        sensorWire.beginWrite(0x51);
        sensorWire.write(0);
        sensorWire.endWrite();
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        double currentRuntime = opMode.getRuntime();

        // return the maximum distance if a response hasn't been
        // received yet from the last range reading command
        int distanceRead = 765;

        // wait a specified amount of time before requesting another range reading
        if(currentRuntime - this.lastRangeReading > RANGE_READING_COMMAND_DELAY_MS) {
            this.lastRangeReading = currentRuntime;

            // request to read the high and low range bytes
            sensorWire.requestFrom(0, 2);

            // write the actual command (0x51) to request a range reading
            takeRangeReading();
        }

        // check if a response has been received
        if(sensorWire.responseCount() > 0) {
            sensorWire.getResponse();
            if(sensorWire.isRead()) {
                // read the high and low bytes of the reported distance
                distanceRead = sensorWire.readHL();
            }
        }

        return distanceRead;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MaxBotix MB1043 HRLV-MaxSonar-EZ4 Range Sensor";
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
        sensorWire.close();
    }
}

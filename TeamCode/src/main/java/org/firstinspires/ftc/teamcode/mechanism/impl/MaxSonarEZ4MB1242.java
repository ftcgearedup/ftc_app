package org.firstinspires.ftc.teamcode.mechanism.impl;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cSensor(name = "MB1043 Range Sensor", description = "Range Sensor from MaxBotix", xmlTag = "MB1242")
public class MaxSonarEZ4MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {

    private static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    @Override
    public double getDistance(DistanceUnit unit) {
        return 0;
    }

    public enum Register {
        FIRST(0),
        RANGE_READING(0x51),
        ADDR_UNLOCK_1(0xAA),
        ADDR_UNLOCK_2(0xA5),
        LAST(ADDR_UNLOCK_2.bVal);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "MaxBotix MB1043 HRLV-MaxSonar-EZ4 Range Sensor";
    }

    private void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.ONLY_ONCE);

        this.deviceClient.setReadWindow(readWindow);
    }

    public MaxSonarEZ4MB1242(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.read(0, I2cDeviceSynch.ReadWindow.READ_REGISTER_COUNT_MAX);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT); // Deals with USB cables getting unplugged

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}

package org.firstinspires.ftc.teamcode.mechanism.impl;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

@I2cSensor(name = "MaxBotix MaxSonarEZ4 Range Sensor", description = "Range Sensor from MaxBotix", xmlTag = "MB1242")
public class MaxSonarEZ4MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

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

        return "MaxBotix MaxSonarEZ4 Range Sensor";
    }

    public MaxSonarEZ4MB1242(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

      //  this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}

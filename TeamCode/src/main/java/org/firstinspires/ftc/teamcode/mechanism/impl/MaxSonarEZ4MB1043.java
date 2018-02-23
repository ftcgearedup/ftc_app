package org.firstinspires.ftc.teamcode.mechanism.impl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class MaxSonarEZ4MB1043 extends MaxSonarEZ4AbstractSensor {
    /**
     * Creates a new MaxSonarEZ4MB1043 instance.
     *
     * @param robot           the robot
     * @param hardwareMapName the name of the sensor in the hardware
     */
    public MaxSonarEZ4MB1043(Robot robot, String hardwareMapName) {
        super(robot, hardwareMapName, DistanceUnit.MM, 5);
    }
}

package org.firstinspires.ftc.teamcode.test;


import static org.junit.Assert.*;
import org.junit.Test;

/**
 * Created by ftc6347 on 9/24/17.
 */

public class HDriveTrigDistanceTest {

    private static final int WHEEL_DIAMETER_INCHES = 4;

    private static final int COUNTS_PER_MOTOR_REV = 1120;

    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Test
    public void mainTest() {
        testHelper(45, 10, 630, 630);
        testHelper(30, 10, 445, 771);
        testHelper(82, 50, 4412, 620);
//        testHelper(62.75, 37, 2931, 1000);
    }

    private void testHelper(double angleDegrees, int targetDistance,
                           int expectedLateral, int expectedAxial) {
        double encoderTargetCounts = COUNTS_PER_INCH * targetDistance;
        double angleRadians = Math.toRadians(angleDegrees);
        int lateralCounts = (int)(encoderTargetCounts * Math.sin(angleRadians));
        int axialCounts = (int)(encoderTargetCounts * Math.cos(angleRadians));

        assertTrue("lateral counts didn't match",
                expectedLateral == lateralCounts);

        assertTrue("axial counts didn't match",
                expectedAxial == axialCounts);
    }
}

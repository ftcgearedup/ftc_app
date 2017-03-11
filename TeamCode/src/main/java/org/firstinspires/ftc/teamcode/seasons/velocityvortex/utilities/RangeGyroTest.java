package org.firstinspires.ftc.teamcode.seasons.velocityvortex.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.LinearOpModeBase;

/**
 * Created by ftc6347 on 3/6/17.
 */
@Autonomous(name = "Range gyro test", group = "utilities")
public class RangeGyroTest extends LinearOpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();

        rangeGyroStrafe(0, 15, -65, -65);
        rangeGyroStrafe(0, 15, 55, 55);
    }
}

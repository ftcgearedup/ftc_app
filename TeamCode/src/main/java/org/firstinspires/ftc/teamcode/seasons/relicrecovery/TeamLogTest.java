package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.TeamLog;

/**
 * Created by peace on 1/28/2018.
 */
@Autonomous (name = "Test Team Log", group = "autonomous" )
public class TeamLogTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeamLog addToLog = new TeamLog("sdcard/log.file");
        addToLog.appendLog("Hello World!", "EXTRA" );
    }
}

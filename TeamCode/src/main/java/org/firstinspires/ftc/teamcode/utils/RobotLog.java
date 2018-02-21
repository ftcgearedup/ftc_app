package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.net.SocketException;

/**
 * Created by peace on 2/20/2018.
 */
@Autonomous(name = "GraphLog", group = "Autonomous")
public class RobotLog extends OpMode {
    @Override
    public void init() {
        try {
            DataGramLogServer dataGramLogServer = new DataGramLogServer();
            dataGramLogServer.start();

        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
    }

}

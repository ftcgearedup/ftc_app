package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

public class JsonRobot extends Robot {
    public JSONConfigOptions optionsMap;

    public JsonRobot(OpMode opMode, String path)
    {
        super(opMode);
        optionsMap = new JSONConfigOptions(path);
    }

    public JSONConfigOptions getOptionsMap() {
        return optionsMap;
    }
}

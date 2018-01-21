package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.usb.serial.SerialPort;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Random;

/**
 * This class is a simple testing program to test any code on.
 */
@TeleOp(name = "Test Op", group = "test")
public class JsonOp extends LinearOpMode {
    private RelicRecoveryRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RelicRecoveryRobot(this);

        waitForStart();
        telemetry.addData("Status", "TestOp Running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("JsonData", robot.getOptionsMap().get("wheelDiam"));
            telemetry.update();
        }
    }
}
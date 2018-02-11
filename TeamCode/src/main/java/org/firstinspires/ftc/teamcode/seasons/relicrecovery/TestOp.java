package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.google.gson.JsonPrimitive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Map;

/**
 * This class is a simple testing program to test any code on.
 */
@TeleOp(name = "Test Op", group = "test")
public class TestOp extends LinearOpMode {
    private JSONConfigOptions optionsMap;

    @Override
    public void runOpMode() throws InterruptedException {
        this.optionsMap = new JSONConfigOptions();

        optionsMap.parseFile(new File(AppUtil.FIRST_FOLDER + "/options.json"));

        waitForStart();
        telemetry.addData("Status", "TestOp Running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Wheel Diameter", optionsMap.retrieveData("wheelDiameter"));
            telemetry.addData("Inside Wheel Gearing Ratio", optionsMap.retrieveData("wheelGearRatioIn"));
            telemetry.addData("Outside Wheel Gearing Ratio", optionsMap.retrieveData("wheelGearRatioOut"));
            telemetry.addData("Jewel Arm Retract Pos", optionsMap.retrieveData("jewelKnockerRetractPosition"));
            telemetry.addData("Jewel Arm Extend Pos", optionsMap.retrieveData("jewelKnockerExtendPosition"));
            telemetry.addData("Relic Grip Open Pos", optionsMap.retrieveData("relicGripOpenPosition"));
            telemetry.addData("Relic Grip Close Pos", optionsMap.retrieveData("relicGripClosePosition"));
            telemetry.addData("Is Right Wheel Reversed?", optionsMap.retrieveData("isRightWheelReversed").toString());
            telemetry.update();
        }
    }
}
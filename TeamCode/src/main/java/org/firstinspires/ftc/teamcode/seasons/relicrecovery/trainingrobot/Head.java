package org.firstinspires.ftc.teamcode.seasons.relicrecovery.trainingrobot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.IMechanism;

/**
 *
 */

public class Head implements IMechanism {
    private CRServo head;


    public Head(Robot robot) {
        HardwareMap hwMap = robot.getCurrentOpMode().hardwareMap;
        this.head = hwMap.crservo.get("head");
    }
    public void setHeadRotationPower(double power) {
        head.setPower(power);
    }
}
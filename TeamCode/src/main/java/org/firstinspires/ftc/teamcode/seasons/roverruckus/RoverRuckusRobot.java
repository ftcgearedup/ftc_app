package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.MecanumDrivetrain;

/**
 * Created by peace on 9/16/2018.
 */

public class RoverRuckusRobot extends Robot {
    private final MecanumDrivetrain mecanumDrivetrain;

    public RoverRuckusRobot(OpMode opMode){
        super(opMode);
        this.mecanumDrivetrain = new MecanumDrivetrain.Builder(this).build();

    }
    public MecanumDrivetrain getMecanumDrivetrain() {
        return mecanumDrivetrain;
    }
}

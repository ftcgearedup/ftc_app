package org.firstinspires.ftc.teamcode.teaching;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.JsonRobot;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanism.drivetrain.impl.DemoDriveTrain;


import org.firstinspires.ftc.teamcode.utils.JSONConfigOptions;

/**
 * This class represents the Relic Recovery robot.
 */
public class DemoRobot extends JsonRobot {

    private final DemoDriveTrain demoDriveTrain;

    //demoDriveTrain is null right now

//   private final JSONConfigOptions optionsMap;

    double testSpeed;

    /**
     * Construct a new Demo robot, with an op-mode that is using this robot.
     *
     * @param opMode the op-mode that this robot is using.
     */
    public DemoRobot(OpMode opMode, String path) {

        super(opMode,path);
//        this.optionsMap = new JSONConfigOptions(path);

        testSpeed = this.optionsMap.retrieveAsDouble("testSpeed");
        //this is an implementation of Json, this line scans through the 'testOptions.json' text
        //file, and retrieves the value that appears right next to it:  "testValue1" : 1,

        this.demoDriveTrain = new DemoDriveTrain.Builder(this)
                .setLeftMotorDirection(DcMotor.Direction.REVERSE)
                .setRightMotorDirection(DcMotor.Direction.FORWARD)
                .build();

        //DemoDriveTrain is located in /teamcode/mechanism/drivetrain/impl

        //
        //in order to use the demoDriveTrain, it can't be 'null', it has to be 'built'. Most people
        //build objects with 'constructors' which appear like this:
        //
        //  Class object1 = new Class(parameter1, parameter2);
        //
        //however, this code here uses a builder pattern, which is just a style choice, and also can
        //be used for a bit of extra functionality, like verifying if the object is constructed
        //correctly, and wont do so until every value is set to something not null
        //

        opMode.telemetry.addData(">", "done initializing");
        opMode.telemetry.update();
    }


    //these 'getter' methods are useful for accessing these private variables, without modifying
    //them
    public DemoDriveTrain getDemoDriveTrain() {
        return demoDriveTrain;
    }
    public double getTestSpeed(){return testSpeed;}
    public JSONConfigOptions getOptionsMap(){return optionsMap;};


}


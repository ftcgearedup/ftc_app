package org.firstinspires.ftc.teamcode.teaching;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by lucas 6-12-19. this teleop is designed to exemplify how to use class structures
 */

@TeleOp(name = "DemoOp", group = "Demo")
public class DemoOp extends LinearOpMode {

    private DemoRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {


        //control-click this to see more!
        robot = new DemoRobot(this,"testing.json");
        //creates new DemoRobot, which has an DemoDriveTrain within it
        //most robot classes have more than just a drive train, many have extra systems classes,
        //which implement IMechanism. such examples includes RelicRecoveryRobot, which has an
        //HDriveTrain, but also a GlyphLift, and a JewelKnocker

        gamepad1.setJoystickDeadzone((float) robot.getOptionsMap().retrieveAsDouble("joystickDeadZone"));
        //it is far easier to just type gamepad1.setJoystickDeadzone(.02f); ,
        // but this way just demonstrates Json stuff

        waitForStart();

        //declare speed and pivot variables, which are faster to write than gamepad1.right_stick_y
        //and so on

        double speedY;
        double pivot;

        //while the program is running
        while (opModeIsActive()) {

            speedY = -gamepad1.left_stick_y;
            pivot = gamepad1.right_stick_x;


            //as you can see here, movement code is super simple, because all of it is handled
            //two classes above, in DemoDriveTrain, you can see how pivot and drive work

            if(gamepad1.right_trigger>.7)
            {
                robot.getDemoDriveTrain().pivot(.75*pivot);
                robot.getDemoDriveTrain().drive(.75*speedY,0);//targetDistance is not needed,
            }
            else if(gamepad1.left_trigger>7)
            {
                robot.getDemoDriveTrain().pivot(.5*pivot);
                robot.getDemoDriveTrain().drive(.5*speedY,0);//targetDistance is not needed,
            }
            else
            {
                robot.getDemoDriveTrain().pivot(pivot);
                robot.getDemoDriveTrain().drive(speedY,0);//targetDistance is not needed,
            }


            robot.getDemoDriveTrain().pivot(pivot);
            robot.getDemoDriveTrain().drive(speedY,0);//targetDistance is not needed,
                                                                    // so we pass zero

            telemetry.addData("what is test speed?" , robot.getTestSpeed());
            //this is just another demonstration of Json

//            telemetry.addData("speedX",speedX);



            telemetry.addData("speedY",speedY);
            telemetry.addData("pivot",pivot);

            telemetry.update();

        }
    }
}



package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.VufTFLiteHandler;

@Autonomous(name = "PreJoystick ", group = "Autonomous")

public class PreJoystick extends LinearOpMode {

    int ground =0;
    int landing =0;
    int clickedbefore = 0 ;

    ElapsedTime tmoe = new ElapsedTime(ElapsedTime.SECOND_IN_NANO);

    String[][] autos = new String[][]  {  {  "LanderStart", "GroundStart" },{  "nothing else", "Depot",  "Crater"} };


    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStarted())
        {

            telemetry.clear();

            clickedbefore = 0;

            while(gamepad1.dpad_up)
            {
                telemetry.addData("",autos[0][landing] +" : "+ autos[1][ground]);
                telemetry.update();


                if(clickedbefore == 0 )
                { landing++; clickedbefore =1;}
                if(landing >1)
                    landing = 0;
            }
            clickedbefore = 0;
            while (gamepad1.dpad_down)
            {
                telemetry.addData("",autos[0][landing] +" : "+ autos[1][ground]);
                telemetry.update();

                if(clickedbefore == 0 )
                { landing--; clickedbefore =1;}
                if(landing <0)
                    landing = 1;
            }

            clickedbefore = 0;
            while(gamepad1.dpad_right)
            {
                telemetry.addData("",autos[0][landing] +" : "+ autos[1][ground]);
                telemetry.update();

                if(clickedbefore == 0 )
                { ground++; clickedbefore =1;}
                if(ground >2)
                    ground = 0;
            }
         clickedbefore = 0;
            while (gamepad1.dpad_left)
            {
                telemetry.addData("",autos[0][landing] +" : "+ autos[1][ground]);
                telemetry.update();
//                telemetry.addData("DpadLEFT",ground);
//                telemetry.update();

                if(clickedbefore == 0 )
                { ground--; clickedbefore =1;}
                if(ground <0)
                    ground = 2;
            }
            telemetry.addData("",autos[0][landing] +" : "+ autos[1][ground]);
            telemetry.update();

        }



        waitForStart();



while(opModeIsActive())
    {
        break;

    }//opmode loop end


    }
}
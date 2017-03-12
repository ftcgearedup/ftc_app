/*
Modern Robotics Color Beacon Example
Created 12/7/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "cb" (MRI Color Beacon with default I2C address 0x4c)

MRIColorBeacon class must be in the same folder as this program. Download from http://modernroboticsinc.com/color-beacon

To change color sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com
*/

package org.firstinspires.ftc.teamcode.seasons.velocityvortex.utilities.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.LinearOpModeBase;


@TeleOp(name="Future Sensors", group="utilities")
//@Disabled
public class FutureSensors extends LinearOpModeBase {



    byte red = 0;     //red value to sent to sensor
    byte green = 0;   //green ...
    byte blue = 0;

    int colorNumber;  //number representing a preset color on the Color Beacon
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            if(gamepad1.b)
                red++;
            if(gamepad1.a)
                green++;
            if(gamepad1.x)
                blue++;
            if(gamepad1.b || gamepad1.a || gamepad1.x)
                getColorbeacon().rgb(red, green, blue);            //Set beacon to illuminate the current red, green, and blue values

            if(gamepad1.y){
                colorNumber++;                      //increase color number
                getColorbeacon().colorNumber(colorNumber);    //sets color number between 0 and 7. If value is >, modulus is used
                sleep(100);
            }

            telemetry.addData("touch", getTouchSensor().isPressed());
            if (getCompassSensor().isCalibrating()) {

                telemetry.addData("compass", "calibrating %s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");

            } else {

                // getDirection() returns a traditional compass heading in the range [0,360),
                // with values increasing in a CW direction
                telemetry.addData("heading", "%.1f", getCompassSensor().getDirection());

                // getAcceleration() returns the current 3D acceleration experienced by
                // the sensor. This is used internally to the sensor to compute its tilt and thence
                // to correct the magnetometer reading to produce tilt-corrected values in getDirection()
                Acceleration accel = getCompassSensor().getAcceleration();
                double accelMagnitude = Math.sqrt(accel.xAccel*accel.xAccel + accel.yAccel*accel.yAccel + accel.zAccel*accel.zAccel);
                telemetry.addData("accel", accel);
                telemetry.addData("accel magnitude", "%.3f", accelMagnitude);

                // getMagneticFlux returns the 3D magnetic field flux experienced by the sensor
                telemetry.addData("mag flux", getCompassSensor().getMagneticFlux());
            }

            // the command register provides status data
            telemetry.addData("command", "%s", getCompassSensor().readCommand());

            telemetry.addData("rgb", (red & 0xFF) + " " + (green & 0xFF) + " " + (blue & 0xFF));
            telemetry.addData("Color", getColorbeacon().getColor());  //getColor() returns a text string with the current color of the beacon
            telemetry.update();
        }
    }
}

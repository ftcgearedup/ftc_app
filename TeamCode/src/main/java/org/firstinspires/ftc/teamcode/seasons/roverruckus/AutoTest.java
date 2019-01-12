package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.Direction;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.VufTFLiteHandler;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.EncoderValues;

// line 44 is where movement starts
@Autonomous(name = "TestingAuto", group = "Autonomous")
//@Disabled
public class AutoTest extends VufTFLiteHandler {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor intakeLift;
    private DcMotor lift;
    private CRServo lBucket;
    //private Servo hook;
    private BNO055IMUWrapper imu;
    private VuforiaNav useVuforia;

    boolean isSampling = true;

    private double ticksPerRevNR20 = 560;
    private double ticksPerRevNR40 = 1120;
    private double ticksPerRevNR60 = 1680;

    //The post gear box gear ratio.
    private double gearRatio = 1.0;
    //The circumference of the drive wheel.
    private double wheelCircumference = 25.1327; // ??
    //Formula to calculate ticks per centimeter for the current drive set up.FORWARDS/BACKWARD ONLY
    private double ticksPerCm = (ticksPerRevNR20 * gearRatio) / wheelCircumference;
    //Formula to calculate ticks per centimeter for the current drive set up.SIDEWAYS

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        initAll();

//        telemetry.addLine("please face robot to 2 leftmost minerals!");
//        telemetry.update();
        waitForStart();
//        telemetry.clear();
        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //unlatch from lander
        getTensorFlowData();
        //may need to back up in order to get all minerals into view

        while (opModeIsActive()) {

            forward(-100,.2);
            sideRight(100,.2);
            pivotCW(1000,.2);


//            sideLeft(20,.1);

            telemetry.addLine("now laterally Aligning");
            telemetry.update();

            lateralAlignToGoldMineral();
            telemetry.clear();
            telemetry.update();
            intake.setPower(1);
            forward(3500,1);
            intake.setPower(0);

            //unlatch from lander

//            alignToLeftmost2Minerals();
//
//            while (isSampling && opModeIsActive()) {
//                getTensorFlowData();
//                if (goldMineralPosition.equals("Left")) {
//                    telemetry.addData("GoldMineralPosition", "Left");
//                    pivotCC(170, .4);
//                    forward(65, .5);
//                    pivotCW(220, .4);
//                    forward(55, .5);
//                    sideRight(10, .5);
//                    pivotCW(1200, .3);
//                    sideLeft(2400, .3);
//                    sideRight(250, .3);
//                    forward(225, .7);
//                    isSampling = false;
//                    break;
//                } else if (goldMineralPosition.equals("Right")) {
//                    telemetry.addData("GoldMineralPosition", "Right");
//                    pivotCW(165, .4);
//                    forward(55, .5);
//                    pivotCC(215, .4);
//                    forward(55, .5);
//                    sideLeft(10, .5);
//                    pivotCW(1200, .3);
//                    sideLeft(2400, .3);
//                    sideRight(250, .3);
//                    forward(225, .7);
//
//                    pivotCW(1200, .3);
//                    sideLeft(2400, .3);
//                    sideRight(250, .3);
//                    forward(225, .7);
//                    break;
//                } else if (goldMineralPosition.equals("Center")) {
//                    telemetry.addData("GoldMineralPosition", "Center");
////                    runIntake(.8);
//                    forward(55, .5);
//                    //turn intake on
//                    forward(40, .75);
//                    pivotCW(1000, .3);
//                    sideLeft(2400, .3);
//                    pivotCW( 30,.7);
//                    sideRight(300, .3);
//                    forward(125,.7);
//                    pivotCC(20,.6);
//                    forward(100,.7);
//
//                    break;
//                } else {
//                    telemetry.addLine("Not Detecting Gold Mineral");
//                    forward(300, 1);
//                    //shimmy around to detect all 3 minerals
//
//                    while (goldMineralPosition.equals("notDetected") && opModeIsActive()) {
//                        pivotCC(5, .3);
//                        pivotCW(5, .3);
//                        pivotCW(5,.3);
//                        pivotCC(5, .3);
//                        getTensorFlowData();
//                        if (!goldMineralPosition.equals("notDetected")) {
//                            forward(200, 1);
//
//                            break;
//                        }
//                    }
//                }
//                getTensorFlowData();
//            } // isSampling loop end01
            telemetry.update();

        }//opmode loop end

        telemetry.update();
    }

    //Methods!!!
    public void readEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowBehv(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeIsActive()) {
            telemetry.addData("Front right encoder ticks: ", frontRight.getCurrentPosition());
            telemetry.addData("Front left encoder ticks: ", frontLeft.getCurrentPosition());
            telemetry.addData("Back right encoder ticks: ", backRight.getCurrentPosition());
            telemetry.addData("Back left encoder ticks: ", backLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    public void initHW() {
        // init the Wheels and other HardWare
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");

//        intake = hardwareMap.dcMotor.get("intake");
//        intakeLift = hardwareMap.dcMotor.get("I-L");


        // set wheel direction
        this.setDirection();
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
//        intake.setPower(0);
//        intakeLift.setPower(0);

    }

    public void setZeroPowBehv(DcMotor.ZeroPowerBehavior behv) {
        frontLeft.setZeroPowerBehavior(behv);
        frontRight.setZeroPowerBehavior(behv);
        backLeft.setZeroPowerBehavior(behv);
        backRight.setZeroPowerBehavior(behv);
    }

    public void setDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    //for backwards use negative power
    //Pass in centimeters.
    public void forward(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double targetDistanceTicks = targetDistance * ticksPerCm;
        double currentDistanceTicks = 0;
        while ((currentDistanceTicks < targetDistanceTicks) && opModeIsActive()) {
            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
            telemetry.update();

            frontLeft.setPower(power);
            frontRight.setPower(power);

            backLeft.setPower(power);
            backRight.setPower(power);
        }
        stopMotors();
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void sideLeft(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while ((currentDistance < targetDistance) && opModeIsActive()) {
            currentDistance = frontLeft.getCurrentPosition();
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

    }

    public void sideRight(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while (currentDistance < targetDistance && opModeIsActive()) {
            currentDistance = frontRight.getCurrentPosition();
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
    }
    //clockwise is 0 cc is 1
    public void pivotCW(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while ((currentDegree < degree) && opModeIsActive()) {
            currentDegree = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
        stopMotors();
    }

    public void pivotCC(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while (currentDegree < degree && opModeIsActive()) {
            currentDegree = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2;
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        stopMotors();
    }
    public void lateralAlignToGoldMineral(){

        getTensorFlowData();
        while(goldMineralX == -1 && opModeIsActive()) {
//            telemetry.addData("goldMineralX", goldMineralX);
//            telemetry.update();
            sideLeft(2,.1);
            getTensorFlowData();
            telemetry.addLine("search Aligning");
            telemetry.update();
        }
        while((goldMineralX <= 360 || goldMineralX >= 370) && opModeIsActive() )
        {
//            telemetry.addData("goldMineralX", goldMineralX);
//            telemetry.update();
            if (goldMineralX<=360)
            {
                sideLeft(1,.1);
            }
            if (goldMineralX>=370)
            {
                sideRight(1,.1);
            }
            getTensorFlowData();


            telemetry.addLine("center Aligning");
            telemetry.update();
        }
        if(goldMineralX>360 && goldMineralX<370)
        {
            return;
        }



    }

}
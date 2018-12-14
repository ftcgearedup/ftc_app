package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.Direction;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.VufTFLiteHandler;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.EncoderValues;

// line 44 is where movement starts
@Autonomous(name = "TestingAuto", group = "Autonomous")
public class AutoTest extends VufTFLiteHandler {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor intakeLift;
    private RevRoboticsCoreHexMotor lift;
    private Servo lBucket;
    //private Servo hook;
    private BNO055IMUWrapper imu;
    private VuforiaNav useVuforia;

    /*
     *   Ticks per rotation for different motor types.
     *
     *   Neverest40(1120)
     *   Neverest20(560)
     *   NeverestOrbital20(560)
     *   Neverest60(1680)
     *
     */
    private double ticksPerRevNR20 = 560;
    private double ticksPerRevNR40 = 1120;
    private double ticksPerRevNR60 = 1680;

    boolean isSampling = true;

    //The post gear box gear ratio.
    private double gearRatio = 1.0;
    //The circumference of the drive wheel.
    private double wheelCircumference = 25.1327; // ??
    //Formula to calculate ticks per centimeter for the current drive set up.FORWARDS/BACKWARD ONLY
    private double ticksPerCm = (ticksPerRevNR40 * gearRatio) / wheelCircumference;
    //Formula to calculate ticks per centimeter for the current drive set up.SIDEWAYS

    // tile is about 60 cm
    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        initAll();
        waitForStart();
        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getTensorFlowData();

        while (opModeIsActive()) {
/*
            //while (isSampling && opModeIsActive()) {
                getTensorFlowData();
                if (goldMineralPosition.equals("Left")) {
                    telemetry.addData("GoldMineralPosition", "Left");
                    pivotCC(170, .4);
                    forward(55, .5);
                    //set intake in
                    pivotCW(220, .4);
                    forward(55, .5);
                    sideRight(10, .5);

                    isSampling = false;
                    alignAndDriveToCrater();
                    break;
                } else if (goldMineralPosition.equals("Right")) {
                    telemetry.addData("GoldMineralPosition", "Right");
                    pivotCW(165, .4);
                    forward(55, .5);
                    //set intake in
                    pivotCC(215, .4);
                    forward(55, .5);
                    sideLeft(10, .5);

                    isSampling = false;
                    alignAndDriveToCrater();
                    break;
                } else if (goldMineralPosition.equals("Center")) {
                    telemetry.addData("GoldMineralPosition", "Center");
                    forward(55, .5);
                    //turn intake on
                    forward(40, .75);
                    isSampling = false;
                    alignAndDriveToCrater();
                    break;
                } else {
                    telemetry.addLine("Not Detecting Gold Mineral");

                    //shimmy around to detect all 3 minerals

                    while (goldMineralPosition.equals("notDetected") && opModeIsActive()) {
                        pivotCC(5, .3);
                        pivotCW(5, .3);
                        getTensorFlowData();
                        if (!goldMineralPosition.equals("notDetected")) {

                            break;
                        }
                    }
                }
                getTensorFlowData();
//            } // isSampling loop end
*/
//            alignAndDriveToCrater();
            alignAndDriveToCrater();
            sideLeft(10, .3);

            telemetry.update();
        } //opmode loop end

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

//    public void setIntake(String pos)
//    {
//        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(pos.equals("up"))
//        {
//            intakeLift.setTargetPosition(840);
//        }
//        else if(pos.equals("down"))
//        {
//            intakeLift.setTargetPosition(1680);
//        }
//
//        intakeLift.setPower(.6);
//    }
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
            currentDistance = frontRight.getCurrentPosition();
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
            telemetry.addData("currentDistance",currentDistance);
            telemetry.addData("targetDistance", targetDistance);
            telemetry.update();
        }

    }

    public void sideRight(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while (currentDistance < targetDistance && opModeIsActive()) {
            currentDistance = frontRight.getCurrentPosition();
            ;
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
            currentDegree = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2;
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
    public void alignAndDriveToCrater()
    {
        pivotCW(1000, .3);
        //
        sideLeft(20, .3);
        //
        sideRight(3, .3);
        //
        forward(210, .7);
        //
//        forward(80,1);

    }
}
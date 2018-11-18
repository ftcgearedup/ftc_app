package org.firstinspires.ftc.teamcode.seasons.roverruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.Direction;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.EncoderValues;

// line 44 is where movement starts
@Autonomous(name = "MechenumAuto", group = "Autonomous")
public class AutoTest extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
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

    //The post gear box gear ratio.
    private double gearRatio = 1.0;
    //The circumference of the drive wheel.
    private double wheelCircumference = 32.986; // ??
    //Formula to calculate ticks per centimeter for the current drive set up.FORWARDS/BACKWARD ONLY
    private double ticksPerCm = ticksPerRevNR40 * gearRatio / wheelCircumference;
    //Formula to calculate ticks per centimeter for the current drive set up.SIDEWAYS

    // tile is about 60 cm
    @Override
    public void runOpMode() throws InterruptedException {
        //Init methods.
        initHW();
        useVuforia.init();
        useVuforia.activate();
        /*
        //Wait for start for to start the program.
        waitForStart();
        //Remove bot form lander

        // Sampling stage!!!!!!!!!!!!!
        forward(50, .75);
        //Tensor flow
        //use side left or side right to strafe to gold mineral
        //intake gold
        // if at middle, sideRight(25, .75);
        // if at left sideRight(50, .75);

        //Depot stage!!!!!!!!!!!!!!!!!
        sideLeft(60,.75);
        useVuforia.activate();
        pivotCC(45, .75);
        forward(140, 1);
        // spit mineral out of intake,
        pivotCC(5000, .75);
        // dump marker out of l bucket

        //crater stage!!!!!!!!!!!!!!!!
        pivotCW(2000, .75);
        forward(250, 1);

        while (opModeIsActive()){}
        */
    }


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
        // init the Wheels
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");


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

    public void setDriveMode(DcMotor.RunMode modee) {
        frontLeft.setMode(modee);
        frontRight.setMode(modee);
        backLeft.setMode(modee);
        backRight.setMode(modee);
    }

    //for backwards use negative power
    //Pass in centimeters.
    public void forward(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetDistance = targetDistance * ticksPerCm;
        double currentDistance = 0;
        while (currentDistance < targetDistance) {
            telemetry.addData("Target pos ticks: ", targetDistance);
            currentDistance = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistance);
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
        while (currentDistance < targetDistance) {
            currentDistance = frontRight.getCurrentPosition();
            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }

    }

    public void sideRight(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while (currentDistance < targetDistance) {
            currentDistance = frontRight.getCurrentPosition();
            ;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }

    }
    //clockwise is 0 cc is 1
    public void pivotCW(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while (currentDegree < degree) {
            currentDegree = frontRight.getCurrentPosition();
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
    }

    public void pivotCC(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
            while (currentDegree < degree) {
                currentDegree = frontRight.getCurrentPosition();
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
        stopMotors();
        }
    }


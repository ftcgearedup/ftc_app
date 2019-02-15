package org.firstinspires.ftc.teamcode.seasons.roverruckus;


import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.Direction;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.VufTFLiteHandler;
import org.firstinspires.ftc.teamcode.seasons.velocityvortex.EncoderValues;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;


@Autonomous(name = "LanderCrater", group = "Autonomous")
public class LanderCrater extends VufTFLiteHandler {
    //Lander, sampling, Full Crater..... on crater side
        private DcMotor frontRight;
        private DcMotor backRight;
        private DcMotor backLeft;
        private DcMotor frontLeft;
        private DcMotor intake;
        private DcMotor intakeLift;
        private DcMotor lift;
        private Servo lBucket;
        private DcMotor hook;
        private BNO055IMUWrapper imu;
        private VuforiaNav useVuforia;

        public ElapsedTime land = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
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
        telemetry.addLine("unlatching");
        telemetry.update();
        hook.setPower(1);
        land.reset();
        while(land.milliseconds()<= 5400 && opModeIsActive()){
            telemetry.addData("landing", land.milliseconds());
            telemetry.update();
        }
        // pivotCW(15, .5);
        // forward(107,.2);
        hook.setPower(0);
        telemetry.addData("landed", true);
        forward(4,.1);



        telemetry.update();


        pivotCW(800,.2);

        forward(10, .3);
        sideLeft(15, .4);
        telemetry.clear();
        telemetry.update();
        getTensorFlowData();

        telemetry.addLine("now laterally Aligning");
        lateralAlignToGoldMineral();
        intakeLift.setPower(0);
        forward(160,1);

        intake.setPower(0);

    }



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
            intake = hardwareMap.dcMotor.get("intake");
            intakeLift = hardwareMap.dcMotor.get("intakeLift");
            lift = hardwareMap.dcMotor.get("lift");
            lBucket = hardwareMap.servo.get("lbucket");
            hook = hardwareMap.dcMotor.get("hook");


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
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

        public void setLeftwardState(double power)
        {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }

        public void setRightwardState(double power)
        {
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

        public void lateralAlignToGoldMineral(){


            getTensorFlowData();

            setRightwardState(.1);

            while(goldMineralX == -1 && opModeIsActive()) {

                getTensorFlowData();
                telemetry.addLine("search Aligning");
                telemetry.update();
            }

            stopMotors();

            while((goldMineralX <= 360 || goldMineralX >= 370) && opModeIsActive() && goldMineralX != -1)
            {

                if (goldMineralX<=345)
                {
                    setLeftwardState(.1);
                }

                if (goldMineralX>=385)
                {
                    setRightwardState(.1);
                }
                getTensorFlowData();


                telemetry.addLine("center Aligning");
                telemetry.update();
            }
            if(goldMineralX>360 && goldMineralX<370)
            {
                stopMotors();
                telemetry.addLine("aligned with mineral! :)");
                telemetry.update();

                return;
            }



        }

    }
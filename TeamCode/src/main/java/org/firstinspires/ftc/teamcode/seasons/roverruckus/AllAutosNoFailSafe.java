package org.firstinspires.ftc.teamcode.seasons.roverruckus;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.roverruckus.utility.VufTFLiteHandler;


@Autonomous(name = "AllAutosNoFailsafe", group = "Autonomous")
public class AllAutosNoFailSafe extends VufTFLiteHandler {
    //All Autos, in one class! no failsafe
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

    int ground = 0;
    int landing = 0;
    int clickedbefore = 0;

    String[][] autos = new String[][]{{"LanderStart", "GroundStart"}, {"nothing else", "Depot", "Crater"}};

    String selectedAuto = "";

    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        initAll();

        while (!isStarted()) {

            telemetry.clear();

            clickedbefore = 0;

            while (gamepad1.dpad_up) {
                telemetry.addData("", autos[0][landing] + " : " + autos[1][ground]);
                telemetry.update();


                if (clickedbefore == 0) {
                    landing++;
                    clickedbefore = 1;
                }
                if (landing > 1)
                    landing = 0;
            }
            clickedbefore = 0;
            while (gamepad1.dpad_down) {
                telemetry.addData("", autos[0][landing] + " : " + autos[1][ground]);
                telemetry.update();

                if (clickedbefore == 0) {
                    landing--;
                    clickedbefore = 1;
                }
                if (landing < 0)
                    landing = 1;
            }

            clickedbefore = 0;
            while (gamepad1.dpad_right) {
                telemetry.addData("", autos[0][landing] + " : " + autos[1][ground]);
                telemetry.update();

                if (clickedbefore == 0) {
                    ground++;
                    clickedbefore = 1;
                }
                if (ground > 2)
                    ground = 0;
            }
            clickedbefore = 0;
            while (gamepad1.dpad_left) {
                telemetry.addData("", autos[0][landing] + " : " + autos[1][ground]);
                telemetry.update();
//                telemetry.addData("DpadLEFT",ground);
//                telemetry.update();

                if (clickedbefore == 0) {
                    ground--;
                    clickedbefore = 1;
                }
                if (ground < 0)
                    ground = 2;
            }
            selectedAuto = autos[0][landing] + " : " + autos[1][ground];
            if (selectedAuto == "GroundStart : nothing else") {
                telemetry.addLine("invalid Auto, testing Encoders ");
                telemetry.update();
            } else {
                telemetry.addData("", selectedAuto);
            }


            telemetry.update();

        }

        waitForStart();


        if (selectedAuto == "GroundStart : nothing else") {
//            selectedAuto = "GroundStart : Depot";
        }

        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {

            switch (selectedAuto) {

                case "LanderStart : nothing else":
                    land();
                    break;
                case "LanderStart : Depot":
                    land();
                    pivotCW(1250, .2);

                    forward(10, .1);

                    telemetry.clear();
                    telemetry.update();
                    lateralAlignToGoldMineral();
                    forward(130, .7);
                    break;
                case "LanderStart : Crater":
                    land();
                    pivotCW(1250, .2);

                    forward(10, .1);

                    telemetry.clear();
                    telemetry.update();
                    lateralAlignToGoldMineral();
                    forward(160, .7);
                    break;
                case "GroundStart : Depot":
                    forward(10, .1);

                    telemetry.clear();
                    telemetry.update();
                    lateralAlignToGoldMineral();
                    forward(130, .7);
                    break;
                case "GroundStart : Crater":
                    forward(10, .1);

                    telemetry.clear();
                    telemetry.update();
                    telemetry.addLine("PreLatAlign");
                    telemetry.update();
                    lateralAlignToGoldMineral();
                    telemetry.addLine("PostLatAlign");
                    telemetry.update();
                    forward(160, .7);
                    break;
                case "GroundStart : nothingElse":

                        readEncoders();


                    break;

                default:
                    telemetry.addLine("something is wrong with selectedAuto String variable");
                    telemetry.update();
                    break;
            }
            while(opModeIsActive()) {
                telemetry.addLine("finished!");
                telemetry.update();
                stopMotors();
            }


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
//            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
//            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
//            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
//            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
//            telemetry.update();

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
            currentDegree = (frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        stopMotors();
    }

    public void setLeftwardState(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void setRightwardState(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void land() {

        telemetry.addLine("unlatching");
        telemetry.update();
        hook.setPower(1);
        land.reset();
        while (land.milliseconds() <= 5400 && opModeIsActive()) {
            telemetry.addData("landing", land.milliseconds());
            telemetry.update();
        }

        hook.setPower(0);
        telemetry.addData("landed", true);

    }


    public void lateralAlignToGoldMineral() {



        getTensorFlowData();

        setRightwardState(.1);
//        boolean drivingRight  = true;
//        ElapsedTime scrolltime = new ElapsedTime();
//        scrolltime.reset();

        while(goldMineralX==-1 && opModeIsActive())
        {
            setRightwardState(.1);
        }

        stopMotors();


        while (opModeIsActive()) {

//            telemetry.addData("Scolltime",scrolltime.seconds());
//            telemetry.update();


            while ((goldMineralX <= 360 || goldMineralX >= 370)
                    && opModeIsActive() && goldMineralX !=-1 ) {//goldMineral IS seen, but not center


                telemetry.addData("mineral","center Aligning");
                telemetry.update();

                while (goldMineralX <= 345 && opModeIsActive()) {
//            if(goldMineralX <= 345){
                    setLeftwardState(.1);

                    telemetry.addData("setting","LeftwardState");
                    telemetry.update();
                }

                getTensorFlowData();
                while (goldMineralX >= 385 && opModeIsActive()) {
//            if(goldMineralX >=385){
                    setRightwardState(.1);
                    telemetry.addData("setting", "RightwardState");
                    telemetry.update();

                }

                getTensorFlowData();


                telemetry.update();

            }
            telemetry.update();
        }
        if (goldMineralX > 360 && goldMineralX < 370) {
            stopMotors();

            telemetry.addLine("aligned with mineral! :)");
            telemetry.update();
            return;
        }



    }
}


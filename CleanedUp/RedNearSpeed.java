package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RedNearSpeed")

public class RedNearSpeed extends LinearOpMode {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13.6; // in
    public static double MAX_VEL = ((435*TICKS_PER_REV)/60); //Ticks per sec
    public static double MAX_ACCEL = MAX_VEL / 0.4;//Ticker per sec squared, accelrates to full in 800 ms
    public static double MAX_ANG_VEL = Math.toRadians(184);//Math.toRadians(184)
    public static double MAX_ANG_ACCEL = Math.toRadians(184);

    public static double X_MULTIPLIER = .004; // Multiplier in the X direction- ticks to inches
    public static double Y_MULTIPLIER = .0041; // Multiplier in the Y direction- tickes to inches


    private int kpx = 1;
    private int kpV = 2;
    private int kIx = 1;
    private int kPtheta = 16;

    private int tickMark = 2;
    ColorSensor tagScanner, ground;
    private double startHeading = 0;
    DcMotor rightFront,leftFront,rightRear,leftRear, xAxis,yAxis,transfer, lslides ;
    Servo arm;
    CRServo outtake;
    DistanceSensor frontDist;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        xAxis = hardwareMap.get(DcMotor.class, "xAxis");
        yAxis = hardwareMap.get(DcMotor.class, "yAxis");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        arm = hardwareMap.get(Servo.class, "arm");
        outtake = hardwareMap.get(CRServo.class, "outtake");

        tagScanner = hardwareMap.get(ColorSensor.class, "color");
        ground = hardwareMap.get(ColorSensor.class, "ground");

        lslides = hardwareMap.get(DcMotor.class, "slides");
        lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OpenCvWebcam webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        Eileen robot = new Eileen(rightFront, leftFront, rightRear, leftRear, xAxis, yAxis, transfer, lslides, outtake,
                tagScanner, frontDist, imu, ground);

        ContourPipelineRedClose myPipeline = new ContourPipelineRedClose();
        webcam.setPipeline(myPipeline);

        OpenCvWebcam finalWebcam = webcam;
        finalWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Initialization passed ", 1);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Init Failed ", errorCode);
                telemetry.update();

            }

        });


        while (!opModeIsActive()) {
            double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0) ? 360 + rangle : rangle;

            tickMark = myPipeline.getTick();

            startHeading = rangle;


            telemetry.addData("tick mark: ", tickMark);
            telemetry.addData("Circles: ", myPipeline.numCircles);

            telemetry.addData("startheading: ", startHeading);
            telemetry.update();
            continue;
        }

        waitForStart();

        //MOVEMENT STARTS HERE
        robot.omniodo(-24, 16);//Omni to backboard- X is Lateral Distance- right - / left +    ||Y is forward Distance
        robot.turnRight(startHeading - 90);//Turn
        if (tickMark == 1) {
            robot.strafe(3, Y_MULTIPLIER, -1, startHeading - 90);
        }//Strafe to left if indicated
        if (tickMark == 3) {
            robot.strafe(3.75, Y_MULTIPLIER, 1, startHeading - 90);
        }//Strafe to right if indicated
        robot.distance(5,startHeading-90);//Distance Sensor to BB
        /*
        robot.slidesOut();
        sleep(200);
        robot.homeSlides();
         */
        sleep(2000);
        robot.odo(14, X_MULTIPLIER, -1.5, startHeading-90);//Go Backwards
        sleep(200);

        //right branch ------------------------------------------------------------------------------
        if(tickMark == 3){
            robot.strafe(3.75, Y_MULTIPLIER, -1, startHeading - 90);
            robot.colorOdo(1,1, startHeading-90);
            sleep(200);
            robot.odo(1, X_MULTIPLIER, 0.5, startHeading-90);
            robot.spit();
            robot.odo(1, X_MULTIPLIER, 0.5, startHeading-90);
            robot.strafe(20, Y_MULTIPLIER, 1, startHeading - 90);
            robot.odo(15, X_MULTIPLIER, 1, startHeading-90);
        }//revert to middle

        //middle branch-----------------------------------------------------------------------------
        if(tickMark==2){
            robot.stripStrafe(10, Y_MULTIPLIER, -1, startHeading-90, 1);
            robot.spit();
            robot.odo(1, X_MULTIPLIER, 0.5, startHeading-90);
            robot.strafe(31, Y_MULTIPLIER, 1, startHeading-90);
            robot.odo(20, X_MULTIPLIER, 1, startHeading-90);
        }

        //left branch------------------------------------------------------------------------------
        if(tickMark == 1){
            robot.strafe(6, Y_MULTIPLIER, 1, startHeading - 90);
            robot.odo(20, X_MULTIPLIER, -0.4, startHeading-90);
            sleep(200);
            robot.colorOdo(1,1, startHeading-90);
            robot.spit();
            robot.odo(13, X_MULTIPLIER, 1, startHeading-90);
            robot.strafe(20, Y_MULTIPLIER, 1, startHeading - 90);
            robot.odo(15, X_MULTIPLIER, 1, startHeading-90);
        }//revert to middle


    }
}

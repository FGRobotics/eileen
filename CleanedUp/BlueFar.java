package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BlueFar")

public class BlueFar extends LinearOpMode {
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
    private int kPtheta = 50;

    private int tickMark = 2;

    private double startHeading = 0;

    DcMotor rightFront,leftFront,rightRear,leftRear, xAxis,yAxis,transfer, lslides ;
    CRServo outtake, intake;
    Servo intakeAngle;
    ColorSensor tagScanner;
    DistanceSensor frontDist;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor .class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        xAxis = hardwareMap.get(DcMotor.class, "xAxis");
        yAxis = hardwareMap.get(DcMotor.class, "yAxis");

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(CRServo.class, "intake");
        outtake = hardwareMap.get(CRServo.class, "outtake");

        tagScanner = hardwareMap.get(ColorSensor.class, "color");
        intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        lslides = hardwareMap.get(DcMotor.class, "slides");
        lslides.setDirection(DcMotorSimple.Direction.REVERSE);

        lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        //left increases right decreases
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Eileen robot = new Eileen(rightFront, leftFront, rightRear, leftRear, xAxis, yAxis, transfer, lslides, outtake,
                tagScanner, frontDist, imu);
        //OpenCV Pipeline

        ContourPipelineBlueFar myPipeline = new ContourPipelineBlueFar();
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

        while(!opModeIsActive()){
            double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0) ? 360 + rangle : rangle;

            tickMark = myPipeline.getTick();

            startHeading = rangle;


            telemetry.addData("tick mark: ", tickMark);
            telemetry.addData("Circles: ",myPipeline.numCircles);

            telemetry.addData("startheading: ", startHeading);
            telemetry.update();
            continue;
        }





        waitForStart();

        robot.odo(37.5, X_MULTIPLIER, 1, startHeading);//forward

        if(tickMark == 1) { //--------------------------------------------------------------Left mark--------------------

            sleep(200);
            robot.pivotRight(startHeading-50, 1);

            sleep(200);
            robot.spit();//drop
            sleep(200);

            robot.pivotRight(startHeading, -1);

            robot.odo(5, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);

        }

        if(tickMark == 2) {//------------------------------------------------------Middle Mark----------------------------------------------

            sleep(200);
            robot.odo(3, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);
            robot.spit();
            sleep(250);

        }

        if(tickMark == 3) {//-------------------------------------------------------Right Mark---------------------------------------

            sleep(200);
            robot.pivotLeft(startHeading+45, 1);//right
            sleep(200);
            robot.spit();//drop
            sleep(200);

            robot.pivotLeft(startHeading, -1);
            sleep(200);
            robot.odo(5, X_MULTIPLIER, -1, startHeading);


        }

        robot.odo(9, X_MULTIPLIER, 1, startHeading);//forward;
        sleep(200);
        robot.turnLeft(startHeading+90);

        startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//reset straight angle
        startHeading = (startHeading < 0) ? 360 + startHeading : startHeading;//reset straight angle
        /*
        intakeAngle.setPosition(.94);
        robot.odo(15,X_MULTIPLIER,-1,startHeading);
        sleep(700);
        intakeAngle.setPosition(1);
        robot.odo(4,X_MULTIPLIER,1,startHeading);
        transfer.setPower(1);
        intake.setPower(1);
        robot.odo(6, X_MULTIPLIER, -1, startHeading);
        sleep(1000);
        robot.odo(6, X_MULTIPLIER, 1, startHeading);
        transfer.setPower(0);
        intake.setPower(0);
        */

        robot.odo(65, X_MULTIPLIER, 1, startHeading);//forward
        sleep(200);

        if(tickMark == 1){
            robot.strafe(20, Y_MULTIPLIER, -1, startHeading);
        }

        if(tickMark == 2){
            robot.strafe(17, Y_MULTIPLIER, -1, startHeading);
        }

        if(tickMark == 3){
            robot.strafe(14, Y_MULTIPLIER, -1, startHeading);
        }

        robot.odo(8.5, X_MULTIPLIER, 1, startHeading);
        sleep(200);

        robot.distance(4.5, startHeading);
        sleep(200);

        robot.slides(1100, 1);
        sleep(200);
        robot.drop();

        sleep(300);

        robot.homeSlides();
        sleep(200);

        //robot.strafe(20, Y_MULTIPLIER, 1, startHeading);
        //robot.odo(3, X_MULTIPLIER, 1, startHeading);

    }
}

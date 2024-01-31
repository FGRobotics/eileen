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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RedNear")

public class RedNear extends LinearOpMode {
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

    private double startHeading = 0;
    DcMotor rightFront,leftFront,rightRear,leftRear, xAxis,yAxis,transfer, lslides ;
    CRServo outtake;
    ColorSensor tagScanner;
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

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotor.class, "transfer");

        outtake = hardwareMap.get(CRServo.class, "outtake");

        tagScanner = hardwareMap.get(ColorSensor.class, "color");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");

        lslides = hardwareMap.get(DcMotor.class, "slides");
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

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
            telemetry.addData("Circles: ",myPipeline.numCircles);

            telemetry.addData("startheading: ", startHeading);
            telemetry.update();
            continue;
        }

        waitForStart();

        odo(32, X_MULTIPLIER, 1, startHeading);//forward


        //branch out to drop on left
        if (tickMark == 1) {
            sleep(200);
            pivotRight(startHeading - 90);//pivot right
            sleep(200);



            odo(5, X_MULTIPLIER, 1, startHeading-90);//forward
            sleep(200);

            spit();//drop
            sleep(200);



            odo(33, X_MULTIPLIER, 1, startHeading-90);//forward
            sleep(200);
            //CREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEP FORWARD
            distance(4.5, startHeading-90);
            sleep(200);

            colorStrafe(5.25,Y_MULTIPLIER,-1,startHeading-90, (tagScanner.argb() < 180000000));
            sleep(200);

            slides(1100,1);
            sleep(200);

            drop();
            sleep(500);
            slides(0, -1);
            sleep(200);

            strafe(26,Y_MULTIPLIER,1,startHeading-90);
            sleep(200);
            odo(4, X_MULTIPLIER, 1, startHeading-90);//forward

        }



        //branch out to drop on middle
        if (tickMark == 2) {
            sleep(200);
            turnRight(startHeading-180);//flip 180
            sleep(200);

            odo(8, X_MULTIPLIER, 1, startHeading-180);//forward
            sleep(200);

            spit();//drop
            sleep(200);

            odo(6, X_MULTIPLIER, 1, startHeading-180);//forward
            sleep(200);

            turnLeft(startHeading-90);//face the board
            sleep(200);

            odo(24, X_MULTIPLIER, 1, startHeading - 90);//forward
            sleep(200);
            //CREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEP FORWARD
            distance(4.5, startHeading-90);
            sleep(200);
            strafe(2,Y_MULTIPLIER,-1,startHeading-90);
            sleep(200);

            slides(1075,1);
            sleep(200);

            drop();
            sleep(500);
            slides(0, -1);
            sleep(200);

            strafe(20,Y_MULTIPLIER,1,startHeading-90);
            sleep(200);
            odo(4, X_MULTIPLIER, 1, startHeading-90);//forward

        }


        if (tickMark == 3) {
            sleep(200);
            pivotRight(startHeading - 90);//pivot right
            sleep(200);



            odo(24, X_MULTIPLIER, 1, startHeading-90);//forward
            sleep(200);

            spit();//drop
            sleep(200);



            odo(13.85, X_MULTIPLIER, 1, startHeading-90);//forward
            sleep(200);
            //CREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEP FORWARD
            distance(4.5, startHeading-90);
            sleep(200);

            colorStrafe(4.5,Y_MULTIPLIER,1,startHeading-90, (tagScanner.alpha() > 70));
            sleep(200);

            slides(1100,1);
            sleep(200);

            drop();
            sleep(500);
            slides(0, -1);
            sleep(200);

            strafe(20,Y_MULTIPLIER,1,startHeading-90);
            sleep(200);
            odo(4, X_MULTIPLIER, 1, startHeading-90);//forward

        }


    }

    public void odo(double dist, double tickstoinch, int reverse, double straightHeading){
        double ticks = Math.abs(dist/tickstoinch);
        while (Math.abs(xAxis.getCurrentPosition()) < ticks){
            double fix = odoPID(straightHeading);
            double position = Math.abs(xAxis.getCurrentPosition());



            double output = Math.sin((3.14*position)/ticks)+0.4;
            rightFront.setPower(reverse*(0.6*output + fix));
            leftFront.setPower(reverse*(0.5*output - fix));
            leftRear.setPower(reverse*(0.5*output  - fix));
            rightRear.setPower(reverse*(0.6*output + fix));

            telemetry.addData("angle: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
            telemetry.update();
        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void distance(double lowest, double straightHeading){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (frontDist.getDistance(DistanceUnit.INCH) > lowest && timer.seconds() < 3){
            double fix = odoPID(straightHeading);




            rightFront.setPower(0.35 + fix);
            leftFront.setPower(0.3 - fix);
            leftRear.setPower(0.3  - fix);
            rightRear.setPower(0.35 + fix);

            telemetry.addData("angle: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
            telemetry.update();
        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void colorStrafe(double dist, double tickstoinch, int reverse, double straightHeading, boolean condition){
        double ticks = dist/tickstoinch;
        while (Math.abs(yAxis.getCurrentPosition()) < Math.abs(ticks) && !condition){
            double fix = (reverse == -1) ? strafePID(straightHeading) : odoPID(straightHeading);
            rightFront.setPower(-reverse*(0.6 + fix));
            leftFront.setPower(reverse*(0.6-fix));
            leftRear.setPower(-reverse*(0.6  + fix));
            rightRear.setPower(reverse*(0.6+fix));

            telemetry.addData("angle: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
            telemetry.update();
        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void strafe(double dist, double tickstoinch, int reverse, double straightHeading){
        double ticks = dist/tickstoinch;
        while (Math.abs(yAxis.getCurrentPosition()) < Math.abs(ticks)){
            double fix = (reverse == -1) ? strafePID(straightHeading) : odoPID(straightHeading);

            rightFront.setPower(-reverse*(0.6 + fix));
            leftFront.setPower(reverse*(0.6-fix));
            leftRear.setPower(-reverse*(0.6  + fix));
            rightRear.setPower(reverse*(0.6+fix));

            telemetry.addData("angle: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
            telemetry.update();
        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnRight(double angle){
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        angle = imuNormalize(angle, rangle);
        while (Math.abs(angle - rangle) > 1.5){
            if(Math.abs(angle - rangle) <30){
                rightFront.setPower(-0.2);
                leftFront.setPower(0.2);
                leftRear.setPower(0.2);
                rightRear.setPower(-0.2);
            }else{
                rightFront.setPower(-0.4);
                leftFront.setPower(0.4);
                leftRear.setPower(0.4);
                rightRear.setPower(-0.4);
            }


            telemetry.addData("start heading: ", startHeading);
            telemetry.addData("target: ", angle);
            telemetry.addData("error: ", Math.abs(angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;
            telemetry.addData("angle: ",  rangle );
            telemetry.update();

        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public  void pivotLeft(double angle, int direction){
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        angle = imuNormalize(angle, rangle);
        while (Math.abs(angle - rangle) > 1){
            rightFront.setPower(0);
            leftFront.setPower(-0.4*direction);
            leftRear.setPower(-0.4*direction);
            rightRear.setPower(0);

            telemetry.addData("start heading: ", startHeading);
            telemetry.addData("target: ", angle);
            telemetry.addData("error: ", Math.abs(angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;
            telemetry.addData("angle: ",  rangle );
            telemetry.update();
        }
        leftFront.setPower(0);
        leftRear.setPower(0);


        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double imuNormalize(double angle, double rangle){
        double heading = angle;
        heading = (heading < 0.0) ? heading + 360 : heading;
        heading = (heading > 360.0) ? heading - 360 : heading;

        return heading;
    }
    public double strafePID(double angle){
        angle = imuNormalize(angle, angle);
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        double error = (rangle > angle) ? rangle - angle : angle - rangle;
        error = error/180;

        return (error*kPtheta)/10;


    }
    public double odoPID(double angle){
        angle = imuNormalize(angle, angle);
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        double error = angle - rangle;
        error = error/180;

        return (error*kPtheta)/10;


    }
    public void spit(){
        transfer.setPower(-1);
        sleep(1200);
        transfer.setPower(0);
    }
    public void slides(int target, int reverse){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(reverse*lslides.getCurrentPosition() < Math.abs(target) &&timer.seconds() < 3){

            lslides.setPower(reverse*0.7);
        }
        lslides.setPower(0);

    }
    public void drop(){
        outtake.setPower(-0.5);
        sleep(2000);
        outtake.setPower(0);
    }
    public  void pivotRight(double angle){
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        angle = imuNormalize(angle, rangle);
        while (Math.abs(angle - rangle) > 1){
            rightFront.setPower(-0.4);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(-0.4);

            telemetry.addData("start heading: ", startHeading);
            telemetry.addData("target: ", angle);
            telemetry.addData("error: ", Math.abs(angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;
            telemetry.addData("angle: ",  rangle );
            telemetry.update();
        }
        rightFront.setPower(0);
        rightRear.setPower(0);


        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnLeft(double angle){

        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        angle = imuNormalize(angle, rangle);
        while (Math.abs(angle - rangle) > 1){
            if(Math.abs(angle - rangle) <30){
                rightFront.setPower(0.2);
                leftFront.setPower(-0.2);
                leftRear.setPower(-0.2);
                rightRear.setPower(0.2);
            }else{
                rightFront.setPower(0.4);
                leftFront.setPower(-0.4);
                leftRear.setPower(-0.4);
                rightRear.setPower(0.4);
            }

            telemetry.addData("start heading: ", startHeading);
            telemetry.addData("target: ", angle);
            telemetry.addData("error: ", Math.abs(angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;
            telemetry.addData("angle: ",  rangle );
            telemetry.update();

        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void homeSlides(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(lslides.getCurrentPosition() > 0 && timer.seconds() < 3){

            lslides.setPower(-1);
        }
        lslides.setPower(0);
        lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RedLeft")

public class RedRight extends LinearOpMode {
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
    private BNO055IMU imu;
    public double motionProfilePos(int inchs, double ticks_sec, double ticks_secsquared, double time){
        // x = vt + 1/2 a t^2
        double dist = inchs / X_MULTIPLIER;
        double acceleration_time = ticks_sec / ticks_secsquared; // max velo / max accel
        double halfway_dist = dist/2;
        double accelDist = 0.5 * ticks_secsquared * (acceleration_time * acceleration_time); // 1/2 * a * t^2
        if(accelDist>halfway_dist){ //if acceleration distance is > halfway
            acceleration_time = Math.sqrt(dist/ticks_secsquared); // change time to be halfway
            accelDist = 0.5 * ticks_secsquared * (acceleration_time * acceleration_time);  //dist = 1/2 * max accel * time^2
        }
        double max_velocity = ticks_secsquared * acceleration_time; //adjust max velo to max accel * accel time
        double decelTime = acceleration_time; //decel time = accel time

        double cruise_dist = dist - (2*accelDist);//cruise dist = dist - accel dist - decel dist
        double cruise_time = dist/max_velocity;//cruise time = dist / max velo

        double trajTime = cruise_time+acceleration_time+decelTime; //total trajectory time = cruise + accel + decel
        if(time > trajTime || time < 0){//if past traj, stop
            return trajTime;
        }
        if(time < acceleration_time){//accell
            //x = 1/2 * a * t^2
            return 0.5 * ticks_secsquared * (time*time);//Kinematic
        }
        else if(time < acceleration_time+cruise_time){//cruise
            //x = v*t
            return accelDist + max_velocity * (time - acceleration_time);
        }
        else{//decel
            double decel_current_time = time - cruise_time - acceleration_time;
            // x = v*t + 1/2 * a * t^2
            return accelDist + cruise_dist + max_velocity*decel_current_time - 0.5 * ticks_secsquared * (decel_current_time*decel_current_time);
        }


    }
    public double motionProfileVelo(int inchs, double ticks_sec, double ticks_secsquared, double time){
        // x = vt + 1/2 a t^2
        double dist = inchs / X_MULTIPLIER;
        double acceleration_time = ticks_sec / ticks_secsquared; // max velo / max accel
        double halfway_dist = dist/2;
        double accelDist = 0.5 * ticks_secsquared * (acceleration_time * acceleration_time); // 1/2 * a * t^2
        if(accelDist>halfway_dist){ //if acceleration distance is > halfway
            acceleration_time = Math.sqrt(dist/ticks_secsquared); // change time to be halfway
            accelDist = 0.5 * ticks_secsquared * (acceleration_time * acceleration_time);  //dist = 1/2 * max accel * time^2
        }
        double max_velocity = ticks_secsquared * acceleration_time; //adjust max velo to max accel * accel time
        double decelTime = acceleration_time; //decel time = accel time

        double cruise_dist = dist - (2*accelDist);//cruise dist = dist - accel dist - decel dist
        double cruise_time = dist/max_velocity;//cruise time = dist / max velo

        double trajTime = cruise_time+acceleration_time+decelTime; //total trajectory time = cruise + accel + decel
        if(time > trajTime || time < 0){//if past traj, stop
            return trajTime;
        }
        if(time < acceleration_time){//accell
            //v =  a * t
            return time * ticks_secsquared;//Kinematic
        }
        else if(time < acceleration_time+cruise_time){//cruise
            //v = v
            return max_velocity;
        }
        else{//decel
            double decel_current_time = time - cruise_time - acceleration_time;
            // v = v - at
            return max_velocity - ticks_secsquared*decel_current_time;
        }


    }
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

        outtake = hardwareMap.get(CRServo.class, "outtake");


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

        ContourPipelineRed myPipeline = new ContourPipelineRed();
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

            /*
            if(gamepad1.right_bumper){
                myPipeline.lowerHSV.val[0] += 10;
                sleep(100);
            }
            if(gamepad1.left_bumper){
                myPipeline.lowerHSV.val[0] -= 10;
                sleep(100);

            }
            if(gamepad1.cross){
                myPipeline.upperHSV.val[0] += 10;
                sleep(100);

            }
            if(gamepad1.triangle){
                myPipeline.upperHSV.val[0] -= 10;
                sleep(100);

            }
            if(gamepad1.square){
                myPipeline.lowerHSV.val[2] += 10;
                sleep(100);

            }
            if(gamepad1.circle){
                myPipeline.lowerHSV.val[2] -= 10;
                sleep(100);

            }

             */
            tickMark = myPipeline.tickMark;


            startHeading = rangle;

            telemetry.addData("lower: ", myPipeline.lowerHSV);
            telemetry.addData("upper: ", myPipeline.upperHSV);
            telemetry.addData("tick mark: ", tickMark);
            telemetry.addData("startheading: ", startHeading);
            telemetry.update();
            continue;
        }

        waitForStart();





        sleep(8000);
        if(tickMark == 1){//-------------------------------------------------------Left Mark---------------------------------------
            yAxis.setPower(1);
            sleep(100);
            yAxis.setPower(0);

            odo(35, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);
            pivotRight(startHeading-35);//right
            sleep(200);
            spit();//drop
            sleep(200);

            odo(10, X_MULTIPLIER, 1, startHeading-35);//forward
            startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//reset straight angle
            startHeading = (startHeading < 0) ? 360 + startHeading : startHeading;//reset straight angle

            sleep(200);
            turnRight(startHeading-52);//right
            sleep(200);


            startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//reset straight angle
            startHeading = (startHeading < 0) ? 360 + startHeading : startHeading;//reset straight angle
            odo(50, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);

            strafe(8, Y_MULTIPLIER, 1, startHeading);
            sleep(200);
            odo(8, X_MULTIPLIER, 1, startHeading);

            slides(1450, 1);
            drop();
            sleep(200);
            slides(0, -1);

            strafe(9, Y_MULTIPLIER, -1, startHeading);

        }
        if(tickMark == 3){ //--------------------------------------------------------------right mark--------------------
            yAxis.setPower(-1);
            sleep(100);
            yAxis.setPower(0);
            sleep(100);
            yAxis.setPower(-1);
            sleep(100);
            yAxis.setPower(0);
            sleep(100);
            yAxis.setPower(-1);
            sleep(100);
            yAxis.setPower(0);

            odo(29.6, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);
            pivotLeft(startHeading+59);

            sleep(200);
            spit();//drop
            sleep(200);

            odo(6, X_MULTIPLIER, 1, startHeading+59);//forward

            sleep(200);
            turnRight(startHeading);

            sleep(200);
            odo(15, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);
            turnRight(startHeading-90);

            sleep(200);



            startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//reset straight angle
            startHeading = (startHeading < 0) ? 360 + startHeading : startHeading;//reset straight angle
            odo(60, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);
            strafe(28, Y_MULTIPLIER, 1, startHeading);

            sleep(200);
            odo(12, X_MULTIPLIER, 1, startHeading);
            sleep(200);
            slides(1200, 1);
            drop();
            sleep(500);

            slides(0, -1);

            strafe(30, Y_MULTIPLIER, -1, startHeading);
            odo(3, X_MULTIPLIER,1, startHeading);

        }
        if(tickMark == 2){//------------------------------------------------------Middle Mark----------------------------------------------
            yAxis.setPower(1);
            sleep(100);
            yAxis.setPower(0);
            sleep(100);
            yAxis.setPower(1);
            sleep(100);
            yAxis.setPower(0);

            odo(34, X_MULTIPLIER, 1, startHeading);//forward
            sleep(200);
            spit();
            sleep(200);

            odo(13, X_MULTIPLIER, 1, startHeading);//forward

            sleep(200);
            turnRight(startHeading-89);//right
            sleep(100);


            startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//reset straight angle
            startHeading = (startHeading < 0) ? 360 + startHeading : startHeading;//reset straight angle

            odo(63, X_MULTIPLIER, 1, startHeading);//forward

            sleep(200);
            strafe(25, Y_MULTIPLIER, 1, startHeading);
            sleep(200);

            odo(4, X_MULTIPLIER, 1, startHeading);
            sleep(100);
            slides(1410, 1);
            drop();
            sleep(300);
            slides(0, -1);

            strafe(20, Y_MULTIPLIER, -1, startHeading);
            odo(3, X_MULTIPLIER, 1, startHeading);



        }



    }
    public void movement(int inches, double maxvel, double maxacc){
        double totalTime = motionProfilePos(inches,maxvel, maxacc, -1);
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        double lastPos = 0;
        double cycleTime = 0;
        double integral = 0;
        while(elapsedTime.seconds() < totalTime + 0.8){
            double currentVelo = (xAxis.getCurrentPosition() - lastPos) / (elapsedTime.seconds()-cycleTime);
            double x_pos = motionProfilePos(inches, maxvel, maxacc, elapsedTime.seconds());

            double pPosition = (x_pos-xAxis.getCurrentPosition());
            double pVelocity = (motionProfileVelo(inches,maxvel,maxacc, elapsedTime.seconds()) - currentVelo);
            double motorPower = (pPosition * kpx);

            lastPos = xAxis.getCurrentPosition();
            cycleTime = elapsedTime.seconds();
            integral += pPosition;

            rightFront.setPower(-motorPower);
            leftFront.setPower(-motorPower);
            leftRear.setPower(-motorPower);
            rightRear.setPower(-motorPower);

            telemetry.addData("error: ", pPosition);
            telemetry.addData("ticks: ", xAxis.getCurrentPosition());
            telemetry.addData("velo: ", pVelocity);
            telemetry.update();

        }
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }//problem with this is that robot wayyyy outpaces motion profiler, have to fix by tuning maximum velocity


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
    public void strafe(double dist, double tickstoinch, int reverse, double straightHeading){
        double ticks = dist/tickstoinch;
        while (Math.abs(yAxis.getCurrentPosition()) < Math.abs(ticks)){
            double fix = odoPID(straightHeading);
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
        while (Math.abs(angle - rangle) > 1){
            rightFront.setPower(-0.3);
            leftFront.setPower(0.3);
            leftRear.setPower(0.3);
            rightRear.setPower(-0.3);

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
    public  void pivotLeft(double angle){
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        angle = imuNormalize(angle, rangle);
        while (Math.abs(angle - rangle) > 1){
            rightFront.setPower(0);
            leftFront.setPower(-0.4);
            leftRear.setPower(-0.4);
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
        sleep(800);
        transfer.setPower(0);
    }
    public void slides(int target, int reverse){
        while(reverse*lslides.getCurrentPosition() < target){

            lslides.setPower(-1 * reverse);
        }
        lslides.setPower(0);
        lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}

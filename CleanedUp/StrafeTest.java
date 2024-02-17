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

@Autonomous(name = "StrafeTest")

public class StrafeTest extends LinearOpMode {
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


        while (!opModeIsActive()) {
            double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0) ? 360 + rangle : rangle;


            startHeading = rangle;


            telemetry.addData("startheading: ", startHeading);
            telemetry.update();
            continue;
        }

        waitForStart();

        strafe(40, Y_MULTIPLIER, -1, startHeading);
        sleep(1000);
        strafe(40, X_MULTIPLIER, 1, startHeading);
        sleep(1000);
    }


    public void odo(double dist, double tickstoinch, int reverse, double straightHeading){
        double ticks = dist/tickstoinch;
        while (xAxis.getCurrentPosition() < Math.abs(ticks)){
            double fix = imuPID(straightHeading);
            rightFront.setPower(reverse*0.8 - fix);
            leftFront.setPower(reverse*0.7 + fix);
            leftRear.setPower(reverse*0.7  + fix);
            rightRear.setPower(reverse*0.8 - fix);

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
            double fix = imuPID(straightHeading);
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


    public double imuNormalize(double angle, double rangle){
        double heading = rangle + angle;
        heading = (heading < 0.0) ? heading + 360 : heading;
        heading = (heading > 360.0) ? heading - 360 : heading;

        return heading;
    }

    public double imuPID(double angle){
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        double error = (rangle > angle) ? rangle - angle : angle - rangle;
        error = error/180;

        return (error*kPtheta)/10;


    }

}

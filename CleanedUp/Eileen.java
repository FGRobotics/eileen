package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Eileen {
    DcMotor rightFront,leftFront,rightRear,leftRear, xAxis,yAxis,transfer, lslides ;
    CRServo outtake;
    ColorSensor tagScanner;
    DistanceSensor frontDist;
    BNO055IMU imu;
    private int kPtheta = 16;




    public Eileen(DcMotor rightFront, DcMotor leftFront, DcMotor rightRear, DcMotor leftRear,
                  DcMotor xAxis, DcMotor yAxis, DcMotor transfer, DcMotor lslides, CRServo outtake,
                  ColorSensor tagScanner, DistanceSensor frontDist, BNO055IMU imu){
        //Initialize everything
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        this.rightRear = rightRear;
        this.leftRear = leftRear;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.transfer = transfer;
        this.lslides = lslides;
        this.outtake = outtake;
        this.tagScanner = tagScanner;
        this.frontDist = frontDist;
        this.imu = imu;
    }

    public void sleep(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds()<seconds){
            continue;
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
        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }//Forward
    public void distance(double lowest, double straightHeading){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (frontDist.getDistance(DistanceUnit.INCH) > lowest && timer.seconds() < 3){
            double fix = odoPID(straightHeading);




            rightFront.setPower(0.35 + fix);
            leftFront.setPower(0.3 - fix);
            leftRear.setPower(0.3  - fix);
            rightRear.setPower(0.35 + fix);


        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }//Distance Sensor
    public void colorStrafe(double dist, double tickstoinch, int reverse, double straightHeading, boolean condition){
        double ticks = dist/tickstoinch;
        while (Math.abs(yAxis.getCurrentPosition()) < Math.abs(ticks) && !condition){
            double fix = (reverse == -1) ? strafePID(straightHeading) : odoPID(straightHeading);
            rightFront.setPower(-reverse*(0.6 + fix));
            leftFront.setPower(reverse*(0.6-fix));
            leftRear.setPower(-reverse*(0.6  + fix));
            rightRear.setPower(reverse*(0.6+fix));


        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }//Color Sensor Strafe
    public void strafe(double dist, double tickstoinch, int reverse, double straightHeading){
        double ticks = dist/tickstoinch;
        while (Math.abs(yAxis.getCurrentPosition()) < Math.abs(ticks)){
            double fix = (reverse == -1) ? strafePID(straightHeading) : odoPID(straightHeading);

            rightFront.setPower(-reverse*(0.6 + fix));
            leftFront.setPower(reverse*(0.6-fix));
            leftRear.setPower(-reverse*(0.6  + fix));
            rightRear.setPower(reverse*(0.6+fix));


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



            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;


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


            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;

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
    }//0-360 normalization
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
        sleep(1.2);
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
        sleep(2);
        outtake.setPower(0);
    }
    public  void pivotRight(double angle, int direction){
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;
        angle = imuNormalize(angle, rangle);
        while (Math.abs(angle - rangle) > 1){
            rightFront.setPower(-0.4*direction);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(-0.4*direction);


            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;

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


            rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            rangle = (rangle < 0.0) ? 360 + rangle : rangle;


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

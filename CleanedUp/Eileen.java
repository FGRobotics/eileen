package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Eileen {
    DcMotor rightFront,leftFront,rightRear,leftRear, xAxis,yAxis,transfer, lslides ;
    CRServo outtake;
    ColorSensor tagScanner, ground;
    DistanceSensor frontDist;
    BNO055IMU imu;
    private int kPtheta = 30;
    private int redTot = 700;
    private int blueTot = 700;
    public double X_MULTIPLIER = .004; // Multiplier in the X direction- ticks to inches
    public double Y_MULTIPLIER = .0041; // Multiplier in the Y direction- tickes to inches




    public Eileen(DcMotor rightFront, DcMotor leftFront, DcMotor rightRear, DcMotor leftRear,
                  DcMotor xAxis, DcMotor yAxis, DcMotor transfer, DcMotor lslides, CRServo outtake,
                  ColorSensor tagScanner, DistanceSensor frontDist, BNO055IMU imu, ColorSensor ground){
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
        this.ground = ground;
    }
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
    public void odo(double dist, double tickstoinch, double reverse, double straightHeading){
        double ticks = Math.abs(dist/tickstoinch);
        while (Math.abs(xAxis.getCurrentPosition()) < ticks){
            double fix = strafePID(straightHeading); //negative if left, positive if right
            if(reverse < 0){
                    fix = -strafePID(straightHeading);

            }
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
        while (frontDist.getDistance(DistanceUnit.INCH) > 5.9 && timer.seconds() < 3){
            double fix = strafePID(straightHeading);




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
    public void stripStrafe(double dist, double tickstoinch, int reverse, double straightHeading, double uppertreshold){
        double ticks = dist/tickstoinch;
        while (Math.abs(yAxis.getCurrentPosition()) < Math.abs(ticks) && !diffCheck()){
            double fix = strafePID(straightHeading);
            if(reverse == 1){
                if (fix > 0) { // if angle is greater than robot angle aka tilted right
                    fix = Math.abs(fix);
                    //if robot is pointed right of straight heading, then left side up force is greater than left side downforce
                    //so we reduce the power to left front(up force) and increase power to left rear(down force)

                    rightFront.setPower(-0.35); //Provides force to the right and down - reversed
                    leftFront.setPower((0.35 - fix)); //Provides force to the right and up
                    leftRear.setPower(-(0.35 + fix)); //Provides force to the right and down - reversed
                    rightRear.setPower((0.35));   //provides force to the right and up

                } else {//if angle is less than robot angle aka tilted left
                    fix = Math.abs(fix);
                    //if robot is titled left of straight heading, then left side downforce is greater than left side up force
                    // so we reduce power to left rear(down force) and increase power to left front(up force)

                    rightFront.setPower(-(0.35)); //Provides force to the right and down - reversed
                    leftFront.setPower((0.35 + fix)); //Provides force to the right and up
                    leftRear.setPower(-(0.35 - fix)); //Provides force to the right and down - reversed
                    rightRear.setPower((0.35));   //provides force to the right and up
                }
            }else{
                if (fix > 0) { // if angle is greater than robot angle aka tilted right
                    fix = Math.abs(fix);
                    //if robot is pointed right of straight heading, then left side up force is greater than left side downforce
                    //so we reduce the power to left rear(up force) and increase power to left front(down force)

                    rightFront.setPower(0.35); //Provides force to the left and up - reversed
                    leftFront.setPower(-(0.35 + fix)); //Provides force to the left and down
                    leftRear.setPower((0.35 - fix)); //Provides force to the left and up - reversed
                    rightRear.setPower(-(0.35));   //provides force to the left and down

                } else {//if angle is less than robot angle aka tilted left
                    fix = Math.abs(fix);
                    //if robot is titled left of straight heading, then left side downforce is greater than left side up force
                    // so we reduce power to left front(down force) and increase power to left rear(up force)

                    rightFront.setPower(0.35); //Provides force to the left and up - reversed
                    leftFront.setPower(-(0.35 - fix)); //Provides force to the left and down
                    leftRear.setPower((0.35 + fix)); //Provides force to the left and up - reversed
                    rightRear.setPower(-(0.35));   //provides force to the left and down
                }
            }




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
            double fix = strafePID(straightHeading);
            if(reverse == 1){
                if (fix > 0) { // if angle is greater than robot angle aka tilted right
                    fix = Math.abs(fix);
                    //if robot is pointed right of straight heading, then left side up force is greater than left side downforce
                    //so we reduce the power to left front(up force) and increase power to left rear(down force)

                    rightFront.setPower(-0.6); //Provides force to the right and down - reversed
                    leftFront.setPower((0.6 - fix)); //Provides force to the right and up
                    leftRear.setPower(-(0.6 + fix)); //Provides force to the right and down - reversed
                    rightRear.setPower((0.6));   //provides force to the right and up

                } else {//if angle is less than robot angle aka tilted left
                    fix = Math.abs(fix);
                    //if robot is titled left of straight heading, then left side downforce is greater than left side up force
                    // so we reduce power to left rear(down force) and increase power to left front(up force)

                    rightFront.setPower(-(0.6)); //Provides force to the right and down - reversed
                    leftFront.setPower((0.6 + fix)); //Provides force to the right and up
                    leftRear.setPower(-(0.6 - fix)); //Provides force to the right and down - reversed
                    rightRear.setPower((0.6));   //provides force to the right and up
                }
            }else{
                if (fix > 0) { // if angle is greater than robot angle aka tilted right
                    fix = Math.abs(fix);
                    //if robot is pointed right of straight heading, then left side up force is greater than left side downforce
                    //so we reduce the power to left rear(up force) and increase power to left front(down force)

                    rightFront.setPower(0.6); //Provides force to the left and up - reversed
                    leftFront.setPower(-(0.6 + fix)); //Provides force to the left and down
                    leftRear.setPower((0.6 - fix)); //Provides force to the left and up - reversed
                    rightRear.setPower(-(0.6));   //provides force to the left and down

                } else {//if angle is less than robot angle aka tilted left
                    fix = Math.abs(fix);
                    //if robot is titled left of straight heading, then left side downforce is greater than left side up force
                    // so we reduce power to left front(down force) and increase power to left rear(up force)

                    rightFront.setPower(0.6); //Provides force to the left and up - reversed
                    leftFront.setPower(-(0.6 - fix)); //Provides force to the left and down
                    leftRear.setPower((0.6 + fix)); //Provides force to the left and up - reversed
                    rightRear.setPower(-(0.6));   //provides force to the left and down
                }
            }




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
                rightFront.setPower(-0.3);
                leftFront.setPower(0.3);
                leftRear.setPower(0.3);
                rightRear.setPower(-0.3);
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

        //how much the robot can tilt without resetting
        double tilt_room_left = 360 - angle;
        double tilt_room_right = 0-angle;
        boolean tooLeft = false;
        boolean tooRight = false;
        //normalization
        double rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rangle = (rangle < 0.0) ? 360 + rangle : rangle;

        //error scaling
        //if tilted left = negative
        //if tilted right = positive
        double error = angle-rangle;

        //the robot will never drift more than 90 degrees tilted direction, so if error is bigger than this, we know robot
        //is too far left, it just hit the reset point and went back down to the zero range
        //so instead of returning a negative fix, it will return a positive one
        if(error > tilt_room_left){
            tooLeft = true;
        }
        //too far right that it will return negative instead of positive
        if(error < tilt_room_right){
            tooRight = true;
        }
        //if this is true make negative
        if(tooLeft){
            rangle += 360;
            error = angle - rangle;
        }

        //if this is true make positive
        if(tooRight){
            error = Math.abs(rangle - 360) + angle;

        }
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
        sleep(1.6);
        transfer.setPower(0);
    }
    public void slidesOut(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(lslides.getCurrentPosition() < Math.abs(1330) &&timer.seconds() < 3){

            lslides.setPower(0.7);
        }
        lslides.setPower(0);

    }
    public void drop(){
        outtake.setPower(-0.5);
        sleep(1.8);
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
                rightFront.setPower(0.3);
                leftFront.setPower(-0.3);
                leftRear.setPower(-0.3);
                rightRear.setPower(0.3);
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
        while(lslides.getCurrentPosition() > 0 && timer.seconds() < 1.5){

            lslides.setPower(-1);
        }
        lslides.setPower(0);
        lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void colorOdo(int upperBound,int lowerBound, double straightHeading){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!diffCheck() && timer.seconds() < 1.1){
            double fix = strafePID(straightHeading);

            rightFront.setPower((0.15+ fix));
            leftFront.setPower((0.2 - fix));
            leftRear.setPower((0.15  - fix));
            rightRear.setPower((0.2 + fix));

        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }//colorODO
    public void updateMeans(){
        redTot = ground.red();
        blueTot = ground.blue();

    }
    public boolean diffCheck(){
        //Checks if current color sensor readings are different than the average value by more than 18%
        int red = ground.red();
        int blue = ground.blue();

        int avgred = redTot;
        int avgblue = blueTot;

        double blueDiff = (blue - avgblue)/avgblue;
        double redDiff = (red - avgred)/avgred;

        if(blueDiff > 0.10 || redDiff > 0.10){
            redTot = 0;
            blueTot = 0;
            updateMeans();
            return true;
        }
        updateMeans();
        return false;

    }
    public void omniodo(int x, int y){ //X is Lateral Distance- right + / left -    ||Y is forward Distance
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        y*= -1;
        x*= -1;
        x /= X_MULTIPLIER;
        y /= Y_MULTIPLIER;
        double theta = Math.atan2(y, x);
        theta = (theta >= 0 && theta < Math.toRadians(270)) ? (-1 * theta) + Math.toRadians(90) : (-1 * theta) + Math.toRadians(450);
        theta = (theta < 0) ? Math.toRadians(360) + theta : theta;
        double mag = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));

        double current_distance_traveled = Math.sqrt(Math.pow(xAxis.getCurrentPosition(),2)+Math.pow(yAxis.getCurrentPosition(),2));
        while (current_distance_traveled < mag){
            double equationone = (Math.sin(theta + (Math.PI / 4)));
            double equationtwo = (Math.sin(theta - (Math.PI / 4)));


            rightFront.setPower((equationone));
            leftRear.setPower((equationone));
            rightRear.setPower((-equationtwo));
            leftFront.setPower((-equationtwo));
            current_distance_traveled = Math.sqrt(Math.pow(xAxis.getCurrentPosition(),2)+Math.pow(yAxis.getCurrentPosition(),2));
        }
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);

        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}

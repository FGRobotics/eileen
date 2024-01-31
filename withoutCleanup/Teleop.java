package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "tele")
public class Teleop extends LinearOpMode{
    private double actfieldCentricMultiplier = .7;
    private double fieldCentricMultiplier = .7;
    private double rotationalMult = .6;
    private int lowerBound = 0;
    double lastResetpos = 0.0;
    boolean isOpen = false;
    boolean intakeOn = false;
    // The IMU sensor object
    public static double X_MULTIPLIER = .004; // Multiplier in the X direction- ticks to inches
    public static double Y_MULTIPLIER = .0041; // Multiplier in th
    BNO055IMU imu;
    ColorSensor tagScanner;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    DcMotor rightFront,leftFront,rightRear,leftRear,lslides,transfer, leds, xAxis, yAxis ;

    Servo intakeAngle, airplane;
    CRServo intake, outtake;
    DistanceSensor frontDist;
    double intakeAngPoses[] = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};
    int intakeAng = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        xAxis = hardwareMap.get(DcMotor.class, "xAxis");
        yAxis = hardwareMap.get(DcMotor.class, "yAxis");

        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        lslides = hardwareMap.get(DcMotor.class, "slides");
        lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lslides.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotor.class, "transfer");

        tagScanner = hardwareMap.get(ColorSensor.class, "color");
        airplane = hardwareMap.get(Servo.class, "airplane");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        intake = hardwareMap.get(CRServo.class, "intake");

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


        waitForStart();
        intakeAngle.setPosition(intakeAngPoses[intakeAng]);

        while (opModeIsActive()) {
            robotCentricPlus();
            gp1();
            gp2();
            telemetry.addData("Slides: ", lslides.getCurrentPosition());
            telemetry.addData("front: ", frontDist.getDistance(DistanceUnit.INCH));

            telemetry.addData("color: ", tagScanner.argb());
            telemetry.addData("color-alpha: ", tagScanner.alpha());


            telemetry.addData("x: ", xAxis.getCurrentPosition()*X_MULTIPLIER);
            telemetry.addData("y: ", yAxis.getCurrentPosition()*Y_MULTIPLIER);

            telemetry.update();
        }
    }
    public void gp1(){
        //Both the actFieldCentricMultipliers cancel out, so we are left with fieldCentricMultiplier being = to the right trigger value
        //It will also have the same sign as the actFieldCentric multiplier, which is the point of the math shown below
        fieldCentricMultiplier = (gamepad1.right_trigger/actfieldCentricMultiplier)*Math.abs(actfieldCentricMultiplier);
        //Invert Joystick
        if (gamepad1.circle) {
            actfieldCentricMultiplier = (actfieldCentricMultiplier == -0.7) ? 0.7 : -0.7;
        }
        if (gamepad1.cross && gamepad1.right_bumper) {
            airplane.setPosition(-1);
            sleep(500);
            airplane.setPosition(0);
            sleep(500);
            airplane.setPosition(-1);

        }
    }
    //-------------------------------------------------------------------------------GP2---------------------------
    public void gp2(){
        //slides
        slides();

        //Outtake

        outtake.setPower(gamepad2.right_bumper ? -0.7 : gamepad2.right_stick_x*0.5);
        //Intake
        intakeAngle.setPosition(intakeAngle.getPosition() + (gamepad2.right_stick_y*0.05));

        intake.setPower(gamepad2.left_bumper  ? -1*gamepad2.right_trigger : gamepad2.right_trigger);
        transfer.setPower(gamepad2.left_bumper  ? -1*gamepad2.right_trigger : gamepad2.right_trigger);



        if(gamepad2.touchpad){
            new Thread(()->{
                intakeAngle.setPosition(1);
            }).start();
        }

        xAxis.setPower(gamepad2.cross ? gamepad2.left_trigger : -gamepad2.left_trigger);
    }
    public void robotCentricPlus() {
        double x, y, mag, rads, rangle;

        fieldCentricMultiplier = (fieldCentricMultiplier == 0) ? actfieldCentricMultiplier : fieldCentricMultiplier;
        x = fieldCentricMultiplier * (gamepad1.left_stick_x);
        y = fieldCentricMultiplier * (gamepad1.left_stick_y);

        double rotational = gamepad1.right_stick_x * rotationalMult;

        //find the resultant vector of the joystick
        mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        //angle of the vector
        rads = Math.atan2(y, x);

        rads = (rads >= 0 && rads < Math.toRadians(270)) ? (-1 * rads) + Math.toRadians(90) : (-1 * rads) + Math.toRadians(450);
        //makes joystick angles go from 0(which is pointing the joystick straight up) to 180 back down to 0

        //find the robot angle
        rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        //find angle robot needs to turn in order to reach joystick angle


        //normalizes two angles
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;
        //subtracts reset position
        rads = rads - lastResetpos;
        rangle = rangle - lastResetpos;
        //turns those back positive
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;
        //calculates the distance between joystick and heading as a positive int
        //double turn = (rads < rangle) ? (Math.toRadians(360) - rangle) + (Math.abs(0 - rads)) : rads - rangle;
        double turn = rads;
        double equationone = (Math.sin(turn + (Math.PI / 4)) * mag);
        double equationtwo = (Math.sin(turn - (Math.PI / 4)) * mag);


        rightFront.setPower((equationone + rotational));
        leftRear.setPower((equationone - rotational));
        rightRear.setPower((-equationtwo + rotational));
        leftFront.setPower((-equationtwo - rotational));


    }
    public void slides(){
        lslides.setPower(gamepad2.left_stick_y*0.7);

    }


}

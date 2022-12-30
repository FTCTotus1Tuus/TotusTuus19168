package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.*;
import java.util.*;
import java.io.*;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.hardware.bosch.BNO055IMU;


@Autonomous(name="RotationTest",group="Linear Opmode") 
public class RotationTest extends LinearOpMode {
    //must be name of file
     // ALL VARIABLES IN THIS CHUNK ARE GLOBAL 
    //Initialize Variables
    public final double SCALE_FACTOR = 255;

    // For Movement
    // int, float, double 
    public double[] direction = {0.0,0.0};
    public double rotation;
    
       public int encoderPos =0;
       public int encoderPos0,encoderPos1,encoderPos2,encoderPos3 =0;
       //public int encoderPos1 =0;
       //public int encoderPos2 =0;
    //   public int encoderPos3 =0;
      public int ZencoderPos =0;
       public double cos = Math.cos((Math.PI)/4);
       public double constMult = (48 * 2 * (Math.PI));
       public double constant = 537.7 / constMult;
       public double rotateDirection;
        private double  targetHeading;
        static final double     P_TURN_GAIN            = 0.03; 
        static final double     P_DRIVE_GAIN           = 0.04;  
        static final double     TURN_SPEED              = 0.2;
        static final double     HEADING_THRESHOLD       = 1.0;   
        private BNO055IMU       imu;    
        private double headingError;
        private double  turnSpeed;
        private double robotHeading;
        private double headingOffset;
       
       public int tileDist = 600;
       //600?
       //changed to 600 to move 1 tile
       public int armDist = 600;
       //set arm dist to 600 as well
    // INITIALIZATION ROBOT PARTS

    // Motor 
    public DcMotor omniMotor0; // front left
    public DcMotor omniMotor1; // front right
    public DcMotor omniMotor2; // back left
    public DcMotor omniMotor3; // back right
    
    
    //public DcMotor omnimotor4 = initializeMotor("linearExtender");    
    public DcMotor linearExtender; // claw height
    
    public CRServo grabServo; // claw open
    public CRServo wristServo; // claw direction
    
    public ColorSensor colorSensor0;
    
    public double match_start_time;
    
    public int parkPos;
   
    
        // initColorSensors();
        // initColorSensors();
    
   public void runOpMode() {
            
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        linearExtender = initializeMotor("linearExtender");
        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        grabServo = hardwareMap.get(CRServo.class, "grabServo");
        wristServo = hardwareMap.get(CRServo.class, "wristServo");
        
        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        linearExtender.setDirection(DcMotor.Direction.REVERSE);

        //test/init telemetry
        print("Motors Init","");
        
        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED
        Rotate(270);
        //while (omniMotor0.isBusy()){}
    //   while (omniMotor0.isBusy()){}
    //     // Rotate(950, 0.33);
    //     // while (omniMotor0.isBusy()){}
    //     // MoveY(-300, 0.15);
    //     wristServo.setPower(-1);
    //     sleep(500);
    //     // MoveZ(0, 0.125);
    // //      MoveY(800,  0.125);
    
    // turnToHeading(0.5, -90);
    
 while (opModeIsActive()){        
    
    this.telemetry.addData("Gyro", Double.toString(getRawHeading()));
    this.telemetry.addData("TargetPos", Integer.toString(ZencoderPos));
    this.telemetry.addData("encoder", Integer.toString(linearExtender.getTargetPosition()));
    this.telemetry.addData("alpha", Integer.toString(colorSensor0.alpha()));
    this.telemetry.addData("blue: ", Integer.toString(colorSensor0.blue()));
    this.telemetry.addData("green: ", Integer.toString(colorSensor0.green()));
    this.telemetry.addData("red: ", Integer.toString(colorSensor0.red()));
    this.telemetry.addData("hue: ", Integer.toString(colorSensor0.argb()));
    this.telemetry.update();
     
    
 }   }
 
 
    public int getParkPos()
    {
            if (colorSensor0.red() > colorSensor0.blue() && colorSensor0.red() > (colorSensor0.green()/1.5))
            {
                telemetry.addData("color", "red");
                telemetry.update();
                return 2;
            } 
            else if (colorSensor0.blue() > colorSensor0.red() && colorSensor0.blue() > (colorSensor0.green()/1.5))
            {
                telemetry.addData("color", "blue");
                telemetry.update();
                return 3;
            } 
            else
            {
                telemetry.addData("color", "green");
                telemetry.update();
                return 1;
            } 
    }
    public void setToRotateRunMode(){
        omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
     public void Rotate(int heading)
    {   
    telemetry.addData("starting rotate function", "");
    telemetry.update();
    boolean isRotating = true;
    if (getRawHeading() - heading > 0){
        rotateDirection = 1;
    }
    else {
        rotateDirection = -1;
    }
        setToRotateRunMode();
        setRotatePower(0.35, rotateDirection);
    while (isRotating){
        
        
        if (Math.abs(5.75 + getRawHeading() - heading) <= 5.75) {
            // resetTargetRotPos();
            encoderPos0 = omniMotor0.getCurrentPosition();
            encoderPos1 = omniMotor1.getCurrentPosition();
            encoderPos2 = omniMotor2.getCurrentPosition();
            encoderPos3 = omniMotor3.getCurrentPosition();
            setRunMode();
            omniMotor0.setTargetPosition(encoderPos0);
            omniMotor1.setTargetPosition(encoderPos1);
            omniMotor2.setTargetPosition(encoderPos2);
            omniMotor3.setTargetPosition(encoderPos3);
            isRotating = false;
        telemetry.addData("finised function 1", "");
        telemetry.update();    
            
        }
    }
        
    }
    public void setRotatePower(double power, double direction){
        omniMotor0.setPower(direction*power);
        omniMotor1.setPower(-direction*power);
        omniMotor2.setPower(direction*power);
        omniMotor3.setPower(-direction*power);
        
    }
    // maybe change V to Z?
    // yessir        /
    //              \/
    public void MoveZ(int z, double power){
       
       
       
       linearExtender.setTargetPosition(z);
       linearExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       linearExtender.setPower(power);
       
   }
   
   public void MoveY(int y, double power){
       
       resetEncoder();
       encoderPos = (int) Math.floor((y * constant+ 0.5));
       setTargetPosY();
       
       setRunMode();
       
       setPower(power);
   }
   
   
      public void MoveX(int x, double power){
       resetEncoder();
       
       encoderPos = (int) Math.floor((x * constant) + 0.5);
        
       setTargetPosX();
       
       setRunMode();
       
       setPower(power);
       //power = 0;
       //setPower(power);
   }
   
   
   public void setTargetPosY()
   {
        
       omniMotor0.setTargetPosition(encoderPos);
       omniMotor1.setTargetPosition(encoderPos);
       omniMotor2.setTargetPosition(encoderPos);
       omniMotor3.setTargetPosition(encoderPos);
       
   }
   
      public void setTargetPosX()
   {
        
       omniMotor0.setTargetPosition(-encoderPos);
       omniMotor1.setTargetPosition(encoderPos);
       omniMotor2.setTargetPosition(encoderPos);
       omniMotor3.setTargetPosition(-encoderPos);
       
   }
    public void setTargetPosRot()
    {
       omniMotor0.setTargetPosition(encoderPos);
       omniMotor1.setTargetPosition(-encoderPos);
       omniMotor2.setTargetPosition(encoderPos);
       omniMotor3.setTargetPosition(-encoderPos);
    }
   

   
    public void setRunMode()
   {
        omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

   }
   
   public void setPower(double power)
   {
       omniMotor0.setPower(power);
       omniMotor1.setPower(power);
       omniMotor2.setPower(power);
       omniMotor3.setPower(power);
       
   }
   
   public void resetEncoder() 
   {
    omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   }
   

   
   
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle >= 0){
            return angles.firstAngle;
       
        }
        else {
            return angles.firstAngle + 361;
        }
    }
    
    //  public double getMatchTime(){
    //     return this.time - match_start_time;
    // }
    
    // public boolean isEndGame(){
    //     if(getMatchTime() < 90){
    //         return false;
    //     }
    //     return true;
    // }
    
    int move_to_position;
    double y;
    
    public void print(String Name, String message)
    {
        telemetry.addData(Name, message);
        telemetry.update();
    }

    public double relative_power(double intended_power)
    {
        return (13 * intended_power) / 13; //getVoltage()
    }
    
    boolean going_to_interpolate = false;
    double interpolation_amount = 0.25;
    
    
   public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         return motor;
     }
     
    public Servo initializeServo(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        Servo motor = hardwareMap.get(Servo.class, name);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         return motor;
     }

     public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
     }
     public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
        }
        
    // boolean timeBetween(double startTime, double endTime){
    //     if((this.time  >= startTime) && (this.time <= endTime)){
    //         return true;
    //     }
    //     return false; 
    // }



}

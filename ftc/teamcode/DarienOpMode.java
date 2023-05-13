package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// @Autonomous(name="DarienOpMode",group="NonCompete")

public class DarienOpMode extends LinearOpMode{
              // ^ must be name of file
     
    //Initialize Variables

    // For Movement
    // int, float, double 
    public double[] direction = {0.0,0.0};
    public int encoderPos =0;
    public int ZencoderPos =0;
    public double rotation;
    public double[] offset = {0.0, 0.0};
    
    public double[] coords = {0.0,0.0};
        // ODOMETRY
            public double cos = Math.cos((Math.PI)/4);
            public double constMult = (48 * 2 * (Math.PI));
            public double constant = 537.7 / constMult;
            
    
    // GYRO ROTATION
        private BNO055IMU       imu;  
        public int encoderPos0,encoderPos1,encoderPos2,encoderPos3 =0;
        public double rotateDirection;
        
    
    // MOVEMENT CONSTANTS
        public int insideTileDist = 578; // Distance between the inside edges of the tile
        public int tileDist = 600;
        public int robotLength = 430;
        public int robotCenterAtStart = insideTileDist/2 - robotLength/2; //Distance to the center of the first tile at start
        public double autoPower = .15;
        public double armPower = .75;
        public int conesMax = 2;
        public boolean interrupt = true;
    
    // INITIALIZATION OF ROBOT PARTS

        // WHEEL MOTORS 
            public DcMotor omniMotor0; // front left
            public DcMotor omniMotor1; // front right
            public DcMotor omniMotor2; // back left
            public DcMotor omniMotor3; // back right
            
            public int rotAmounts0 = 0;
            public int rotAmounts1 = 0;
            public int rotAmounts2 = 0;
            public int rotAmounts3 = 0;
    
        // ARM/CLAW MOTORS
            public DcMotor linearExtender; // claw height
            public CRServo grabServo; // claw open
            public CRServo wristServo; // claw direction
    public int rotationCount;
    
    // TAPE SENSING
            
        //public ColorSensor tapeSensor;
    
    // PARKING
    
        public ColorSensor colorSensor0;
        public int parkPos;
   
   public void initialize() {
       
       
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        linearExtender = initializeMotor("linearExtender");
        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        //tapeSensor = hardwareMap.get(ColorSensor.class, "tapeSensor");
        grabServo = hardwareMap.get(CRServo.class, "grabServo");
        wristServo = hardwareMap.get(CRServo.class, "wristServo");
        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        wristServo.setDirection(CRServo.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        linearExtender.setDirection(DcMotor.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu"); 
        imu.initialize(parameters);

        //test/init telemetry
        print("Motors Init","");
   }    
    public void runOpMode(){
        //Necessary emptpy method for linear op mode
    }


     public void Rotate(double heading, double dir)
    {   
    telemetry.addData("starting rotate function", "");
    telemetry.update();
    double startEncPos0 = omniMotor0.getCurrentPosition();
    double startEncPos1 = omniMotor1.getCurrentPosition();
    double startEncPos2 = omniMotor2.getCurrentPosition();
    double startEncPos3 = omniMotor3.getCurrentPosition();
    boolean isRotating = true;
    rotateDirection = dir;
    setToRotateRunMode();
    
    while (isRotating){
        if (dir == 0){
            if (getRawHeading() - heading > 0){
                rotateDirection = -1;
        }
            else 
            {
                rotateDirection = 1;
        }
    }
    
        telemetry.addData(Double.toString(getRawHeading() - heading), "");
        telemetry.addData(Double.toString(heading), "");
        telemetry.addData(Double.toString(getRawHeading()), "");
        telemetry.update();    
        if (Math.abs(getRawHeading() - heading) <= 2.25)
            {
            telemetry.addData(Boolean.toString(getRawHeading() > heading && dir == 1), " dir = 1");
            telemetry.addData(Boolean.toString(getRawHeading() < heading && dir == -1), "dir = -1");
            // resetTargetRotPos();
            encoderPos0 = omniMotor0.getCurrentPosition();
            encoderPos1 = omniMotor1.getCurrentPosition();
            encoderPos2 = omniMotor2.getCurrentPosition();
            encoderPos3 = omniMotor3.getCurrentPosition();
            rotAmounts0 += encoderPos0 - startEncPos0;
            rotAmounts1 += encoderPos1 - startEncPos1;
            rotAmounts2 += encoderPos2 - startEncPos2;
            rotAmounts3 += encoderPos3 - startEncPos3;
            omniMotor0.setTargetPosition(encoderPos0);
            omniMotor1.setTargetPosition(encoderPos1);
            omniMotor2.setTargetPosition(encoderPos2);
            omniMotor3.setTargetPosition(encoderPos3);
            setRunMode();
            isRotating = false;
            telemetry.addData("finised function 1", "");
            }
        else if (Math.abs(getRawHeading() - heading) <= 20){
            setRotatePower(0.18, rotateDirection);
            telemetry.addData("Error less than 30?", "True");
            telemetry.update();    
        } 
        else {
            setRotatePower(0.225, rotateDirection);
    }
        
    }}
    
    
    // public void RotateEncoders(double destination, int dir, double power) {
    //     //init code
    //     rotateDirection = dir;
        
    //     // rotation code
    //     if (dir == 0){
    //         if (getRawHeading() - heading > 0){
    //             rotateDirection = -1;
    //     }
    //         else 
    //         {
    //             rotateDirection = 1;
    //     }}
    //     double encoderPos = 10 * rotateDirection * Math.abs(getRawHeading() - destination); //encoderTicks-a bit * direction * how far to go
    //     setTargetPosRot();
    //     setPower(power);
    //     while(Math.abs(getRawHeading() - destination) > 2){
    //         print("final approach", "");
            
    //     }}
    // //11.02
    
    public void MoveZ(int z, double power){
       
       
       
       linearExtender.setTargetPosition(z);
       linearExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       linearExtender.setPower(power);
       
   }
   
   public void MoveXandY(int targetX, int targetY, double power) {
       int[] ePos0and3 = inverseEncoderPos0and3(targetX, targetY);
       int[] ePos1and2 = inverseEncoderPos1and2(targetX, targetY);
       
       setTargetPosBoth(ePos0and3[0], ePos1and2[0], ePos1and2[1], ePos0and3[1]);
       
       setRunMode();
       
       setPower(power);
   }
   

   
   public void MoveY(int y, double power){
       
       encoderPos = (int) Math.floor((y * constant+ 0.5));
       setTargetPosY();
       
       setRunMode();
       
       setPower(power);
   }
   
   
      public void MoveX(int x, double power){
       
      encoderPos = (int) Math.floor((x * constant*1.111111) + 0.5);
        
       setTargetPosX();
       
       setRunMode();
       
       setPower(power);
   }
   
   
   public void setTargetPosBoth(int ePos0, int ePos1, int ePos2, int ePos3) {
       omniMotor0.setTargetPosition(ePos0);
       omniMotor1.setTargetPosition(ePos1);
       omniMotor2.setTargetPosition(ePos2);
       omniMotor3.setTargetPosition(ePos3);
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
    public void resetTargetRotPos() {
            encoderPos0 = omniMotor0.getCurrentPosition();
            encoderPos1 = omniMotor1.getCurrentPosition();
            encoderPos2 = omniMotor2.getCurrentPosition();
            encoderPos3 = omniMotor3.getCurrentPosition();
            setRunMode();
            omniMotor0.setTargetPosition(encoderPos0);
            omniMotor1.setTargetPosition(encoderPos1);
            omniMotor2.setTargetPosition(encoderPos2);
            omniMotor3.setTargetPosition(encoderPos3);
    }
   

   
    public void setRunMode()
   {
        omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

   }
   
    public void setToRotateRunMode(){
        omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
   public void setPower(double power)
   {
       omniMotor0.setPower(relativePower(power));
       omniMotor1.setPower(relativePower(power));
       omniMotor2.setPower(relativePower(power));
       omniMotor3.setPower(relativePower(power));
       
   }
   
   public void setRotatePower(double power, double direction){
        omniMotor0.setPower(relativePower(direction*power));
        omniMotor1.setPower(relativePower(-direction*power));
        omniMotor2.setPower(relativePower(direction*power));
        omniMotor3.setPower(relativePower(-direction*power));
        
    }
    

    public void waitForMotors()
    {
        while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy()){}
    }
   
    public double[] getCoords(){
        
        double[] coords = {convertEncoderPosX() + offset[0], convertEncoderPosY() + offset[1]};
        return coords; // returns x,y pos of robot in relation to the center of the field in mm
    }
    
    public double convertEncoderPosX(){
     return ((omniMotor0.getCurrentPosition()-rotAmounts0) 
           -(omniMotor1.getCurrentPosition()-rotAmounts1))/2;    
    }
    public double convertEncoderPosY(){
     return ((omniMotor0.getCurrentPosition()-rotAmounts0) 
           +(omniMotor1.getCurrentPosition()-rotAmounts1))/2;    
    }
    
    public int[] inverseEncoderPos0and3(int targetX, int targetY) {
        // motors 0 and 3's conversion function of coords to encoders is x+y=enocder pos, a in the notebook
        double[] currentCoords = getCoords();
        int distanceX = (int) Math.floor(targetX - currentCoords[0] + 0.5);
        int distanceY = (int) Math.floor(targetY - currentCoords[1] + 0.5);
        int coordsToEncoder = distanceX + distanceY;
        
        int encoderOmni0 = coordsToEncoder + rotAmounts0;
        int encoderOmni3 = coordsToEncoder + rotAmounts3;
        int[] ret = {encoderOmni0, encoderOmni3};
        return ret;
    }
    public int[] inverseEncoderPos1and2(int targetX, int targetY) {
        // motors 1 and 2's conversion function of coords to encoders is y-x=enocder pos, b in the notebook
        double[] currentCoords = getCoords();
        int distanceX = (int) Math.floor(targetX - currentCoords[0] + 0.5);
        int distanceY = (int) Math.floor(targetY - currentCoords[1] + 0.5);
        int coordsToEncoder = distanceY - distanceX;
        
        int encoderOmni1 = coordsToEncoder + rotAmounts1;
        int encoderOmni2 = coordsToEncoder + rotAmounts2;
        int[] ret = {encoderOmni1, encoderOmni2};
        return ret;
    }
    
    //y=\frac{\left(a+b\right)}{2}
    //x=\frac{\left(a-b\right)}{2}
    
    
    int move_to_position;
    double y;
    
    public void print(String Name, String message)
    {
        telemetry.addData(Name, message);
        telemetry.update();
    }

    public double relativePower(double intended_power)
    {
        return (13 * intended_power) / getVoltage();
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
        
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle >= 0){
            return AngleUnit.normalizeDegrees(angles.firstAngle);
       
        }
        else {
            return angles.firstAngle + 361;
        }
    }
        
    boolean timeBetween(double startTime, double endTime){
        if((this.time  >= startTime) && (this.time <= endTime)){
            return true;
        }
        return false; 
    }
    public void interrupt()
    {
        while (interrupt = true){}   
    }
    
      
}
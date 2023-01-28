package org.firstinspires.ftc.Archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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



@Autonomous(name="ColorTest",group="NonCompete") 
public class ColorTest extends LinearOpMode {
    //must be name of file
     // ALL VARIABLES IN THIS CHUNK ARE GLOBAL 
    //Initialize Variables
    public final double SCALE_FACTOR = 255;

    // For Movement
    // int, float, double 
    public double[] direction = {0.0,0.0};
    public double rotation;
    
       public int encoderPos =0;
       public int ZencoderPos =0;
       public double cos = Math.cos((Math.PI)/4);
       public double constMult = (48 * 2 * (Math.PI));
       public double constant = 537.7 / constMult;
       
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
        
        //servo0 = initializeServo("servo0");
        //servo0 = hardwareMap.get(Servo.class, "wristServo");
        //grabServo = hardwareMap.get(Servo.class, "grabServo");
   
        //test/init telemetry
        print("Motors Init","");


        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED

    //   MoveZ(-1800, 0.125);
       
    //     MoveY(tileDist, 0.125);
    //     //move to color cone
    //     while (omniMotor0.isBusy()){}
    //     //stop, add color sensing code
    //   //get parking space
       
 
        

    //     Rotate(500, 0.5);
    //     while (omniMotor0.isBusy()){}
    //     //back up to pole
    //     MoveY(-175, 0.125);
    //     while (omniMotor0.isBusy()){}
    //     MoveY(-100, 0.11);
    //     while (omniMotor0.isBusy()){}
    //     // MoveY(5, 0.1);
    //     // while (omniMotor0.isBusy()){}
    //     sleep(100);
    //     //drop cone
    //     grabServo.setPower(0.5);
    //     sleep(100);
    //     //return
    //     grabServo.setPower(-0.1);
    //     MoveY(275, 0.125);
    //     while (omniMotor0.isBusy()){}
    //     Rotate(475, 0.5);
        
    //     while (omniMotor0.isBusy()){}
    //     //GotoParking Space
    //     switch(parkPos)
    //     {
    //         //Green
    //         case 1:
    //             MoveY(-600, 0.25);
    //             break;
    //         //Red
    //         case 2:
    //             MoveY(0, 0.25);
    //             break;
    //         //Blue
    //         case 3:
    //             Rotate(-25, 0.5);
    //             while (omniMotor0.isBusy()){}
    //             MoveY(625, 0.25);
    //             break;
    //     }
    //   while (omniMotor0.isBusy()){}
    //     // Rotate(950, 0.33);
    //     // while (omniMotor0.isBusy()){}
    //     // MoveY(-300, 0.15);
    //     wristServo.setPower(-1);
    //     sleep(500);
        // MoveZ(0, 0.125);
    //      MoveY(800,  0.125);
 while (opModeIsActive()){        
    parkPos = getParkPos();
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
            if (colorSensor0.red() > colorSensor0.blue() && colorSensor0.red() > (colorSensor0.green()/1.2))
            {
                telemetry.addData("color", "red");

                return 2;
            } 
            else if (colorSensor0.blue() > colorSensor0.red() && colorSensor0.blue() > (colorSensor0.green()/1.2))
            {
                telemetry.addData("color", "blue");

                return 3;
            } 
            else
            {
                telemetry.addData("color", "green");

                return 1;
            } 
    }
    public void Rotate(int rotation, double power)
    {
        resetEncoder();
        encoderPos = rotation;
        setTargetPosRot();
        setRunMode();
        setPower(power);
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
   
   
   
//   public void controllerTest()
//   {
//           if(gamepad1.y){
//         this.telemetry.addData("Moving", "North...");
//         MoveY(1 * tileDist,  0.125);
//     };
//     if(gamepad1.a){
//         this.telemetry.addData("Moving", "South...");
//         MoveY(-1 * tileDist,  0.125);
//     }
//     if(gamepad1.b){
//         this.telemetry.addData("Moving", "West...");
//         MoveX(1 * tileDist,  0.125);
//     };
//     if(gamepad1.x){
//         this.telemetry.addData("Moving", "East...");
//         MoveX(-1 * tileDist,  0.125);
//     }
    
//     if(gamepad1.dpad_down){
//         this.telemetry.addData("Moving", "Down...");
//         MoveZ(1 * armDist,  0.125);
//     };
//     if(gamepad1.dpad_up){
//         this.telemetry.addData("Moving", "Up...");
//         MoveZ(-1 * armDist,  0.125);
//     }
    
//     if(gamepad1.dpad_left){
//         this.telemetry.addData("Moving Claw", "open");
//         wristServo.setPosition(-10);
//     };
//     if(gamepad1.dpad_right){
//         this.telemetry.addData("Moving Claw", "close");
//         wristServo.setPosition(10);
//     }
//     if(gamepad1.left_bumper){
//         this.telemetry.addData("Moving Claw", "L");
//         grabServo.setPosition(-10);
//     };
//     if(gamepad1.right_bumper){
//         this.telemetry.addData("Moving Claw", "R");
//         grabServo.setPosition(10);
//     }
//   }
   
     public double getMatchTime(){
        return this.time - match_start_time;
    }
    
    public boolean isEndGame(){
        if(getMatchTime() < 90){
            return false;
        }
        return true;
    }
    
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
        
    boolean timeBetween(double startTime, double endTime){
        if((this.time  >= startTime) && (this.time <= endTime)){
            return true;
        }
        return false; 
    }



}

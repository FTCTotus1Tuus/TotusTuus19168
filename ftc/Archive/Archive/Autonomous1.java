// package org.firstinspires.ftc.Archive;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.util.Hardware;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;

// import com.qualcomm.robotcore.hardware.HardwareMap;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import java.util.Random;
// import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.*;
// import android.graphics.Color;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import java.lang.annotation.Target;
// import com.qualcomm.robotcore.*;
// import java.util.*;
// import java.io.*;
// import com.qualcomm.robotcore.hardware.LED;
// import com.qualcomm.robotcore.hardware.DigitalChannelController;
// import com.qualcomm.robotcore.hardware.DigitalChannel;


// @TeleOp(name="Autonomous1",group="Linear Opmode") 
// public class Autonomous1 extends LinearOpMode {
//     //must be name of file
//     // ALL VARIABLES IN THIS CHUNK ARE GLOBAL 
//     //Initialize Variables
//     public final double SCALE_FACTOR = 255;

//     // For Movement
//     // int, float, double 
//     public double[] direction = {0.0,0.0};
//     public double rotation;
    
//       public int encoderPos =0;
//       public double cos = Math.cos((Math.PI)/4);
//       public double constMult = (48 * 2 * (Math.PI));
//       public double constant = 537.7 / constMult;
       
   
//     // INITIALIZATION ROBOT PARTS

//     // Motor 
//     public DcMotor omniMotor0; // front left
//     public DcMotor omniMotor1; // front right
//     public DcMotor omniMotor2; // back left
//     public DcMotor omniMotor3; // back right
    
//     public double match_start_time;
   
    
//         // initColorSensors();
//         // initColorSensors();
    
//   public void runOpMode() {
            
//         omniMotor0 = initializeMotor("omniMotor0");
//         omniMotor1 = initializeMotor("omniMotor1");
//         omniMotor2 = initializeMotor("omniMotor2");
//         omniMotor3 = initializeMotor("omniMotor3");
        
//         omniMotor0.setDirection(DcMotor.Direction.REVERSE);
//         omniMotor1.setDirection(DcMotor.Direction.FORWARD);
//         omniMotor2.setDirection(DcMotor.Direction.REVERSE);
//         omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        
   
   
//         //test/init telemetry
//         print("Motors Init","");


//         // Wait for the driver to start - must press play -- will run until driver presses 
//         waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED

        
        
        

        


//          //MoveY(800,  0.125);
//  while (opModeIsActive()){        
//     this.telemetry.addData("TargetPos", Integer.toString(encoderPos));
//     this.telemetry.addData("encoder", Integer.toString(omniMotor0.getTargetPosition()));

    
//         MoveY(600,  0.125);
    
//     if(gamepad1.a){
//         this.telemetry.addData("Moving", "South...");
//         MoveY(-600,  0.125);
//     }
//     if(gamepad1.b){
//         this.telemetry.addData("Moving", "West...");
//         MoveX(600,  0.125);
//     };
//     if(gamepad1.x){
//         this.telemetry.addData("Moving", "East...");
//         MoveX(-600,  0.125);
//     }
    
     
//  }   }

   
//   public void MoveY(int y, double power){
//       resetEncoder();
       
//       encoderPos = (int) Math.floor((y * constant+ 0.5));
//       setTargetPosY();
       
//       setRunMode();
       
//       setPower(power);
//   }
   
   
//       public void MoveX(int x, double power){
//       resetEncoder();
       
//       encoderPos = (int) Math.floor((x * constant) + 0.5);
        
//       setTargetPosX();
       
//       setRunMode();
       
//       setPower(power);
//       //power = 0;
//       //setPower(power);
//   }
   
   
//   public void setTargetPosY()
//   {
        
//       omniMotor0.setTargetPosition(encoderPos);
//       omniMotor1.setTargetPosition(encoderPos);
//       omniMotor2.setTargetPosition(encoderPos);
//       omniMotor3.setTargetPosition(encoderPos);
       
//   }
   
//       public void setTargetPosX()
//   {
        
//       omniMotor0.setTargetPosition(-encoderPos);
//       omniMotor1.setTargetPosition(encoderPos);
//       omniMotor2.setTargetPosition(encoderPos);
//       omniMotor3.setTargetPosition(-encoderPos);
       
//   }
   

   
//     public void setRunMode()
//   {
//         omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//   }
   
//   public void setPower(double power)
//   {
//       omniMotor0.setPower(power);
//       omniMotor1.setPower(power);
//       omniMotor2.setPower(power);
//       omniMotor3.setPower(power);
       
//   }
   
//   public void resetEncoder() 
//   {
//     omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//   }
   
   
   
//      public double getMatchTime(){
//         return this.time - match_start_time;
//     }
    
//     public boolean isEndGame(){
//         if(getMatchTime() < 90){
//             return false;
//         }
//         return true;
//     }
    
//     int move_to_position;
//     double y;
    
//     public void print(String Name, String message)
//     {
//         telemetry.addData(Name, message);
//         telemetry.update();
//     }

//     public double relative_power(double intended_power)
//     {
//         return (13 * intended_power) / 13; //getVoltage()
//     }
    
//     boolean going_to_interpolate = false;
//     double interpolation_amount = 0.25;
    
    
//   public DcMotor initializeMotor(String name){
//          /*This is just a handy dandy function which saves a few lines and looks cool,
//          it initializes the motor and it also initializers the motor power logs for this motor*/
//         DcMotor motor = hardwareMap.get(DcMotor.class, name);
//         motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//          return motor;
//      }
     

//      public static double clamp(double val, double min, double max) {
//         return Math.max(min, Math.min(max, val));
//      }
//      public double getVoltage() {
//         return (hardwareMap.voltageSensor.iterator().next().getVoltage());
//         }
        
//     boolean timeBetween(double startTime, double endTime){
//         if((this.time  >= startTime) && (this.time <= endTime)){
//             return true;
//         }
//         return false; 
//     }
    

// }

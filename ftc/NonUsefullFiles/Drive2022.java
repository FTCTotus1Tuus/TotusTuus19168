package org.firstinspires.ftc.NonUsefullFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

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
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawTest",group="NonCompete") // name = filename, group = way to divy up code 
// public class Filename 
public class Drive2022 extends LinearOpMode{
    // ALL VARIABLES IN THIS CHUNK ARE GLOBAL 
    //Initialize Variables
    Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
    Boolean replaying = false;
    final double SCALE_FACTOR = 255;
    // Our transform (Set the values to default values)
    //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d,0d,0d});
    
    // For Movement
    // int, float, double 
    double[] direction = {0.0,0.0};
    double rotation;
    
    // INITIALIZATION ROBOT PARTS
    DistanceSensor scooperDistanceSensor0;
    DistanceSensor scooperDistanceSensor1;
    
    DcMotor ledMotor;
    ColorSensor sensorColor;
    
    // Motor 
    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right
    
    //LinearExtender
    DcMotor linearExtender;
 
    Servo wristServo;
    Servo grabServo;
    
    double targetPosition; // 
    double MAX_ENCODER_POSITION = 100;
        
    double EPSILON = 0.000000001;
    double armMotorOffset = 10;
    double currentPosition = 0;
    
    double armPower;
    
    double previous_block_in_time = 0;
    
    
    double arm_target_position = 0;
    
    double match_start_time;

    // when "play" is pressed, runOpMode function will be ran 
    public void runOpMode() {
     
        /*
        var = hardwareMap.get(class of variable, string in phone)
        must configure in phone before using in code 
        */

        // connecting the motors to the config and setting to a variable name
       
        
        
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        linearExtender = initializeMotor("linearExtender");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");

        // armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        omniMotor0.setDirection(DcMotor.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.REVERSE);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        linearExtender.setDirection(DcMotor.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        // Initialize each dictionary
        // arr = {0,7,8}
        // arr[1] = 2;
        // print (arr) -- {028}
        
        telemetry.update();
        

        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED
        

        match_start_time = this.time; // time until play button is pressed 
       
        
        

        while (this.opModeIsActive()) {
            // hit "X" button to break from loop and stop program
            // negative down, positive up 
            
            // Ternary operations
            
            // ENTER CODE HERE 
            
            // Move servos
            if (gamepad2.a == true) {
                double wristpos = 1;
                telemetry.addData("wrist",wristpos);
                wristServo.setPosition(wristpos);
                
            }
            
            if (gamepad2.b == true) {
                double wristpos = 0;
                telemetry.addData("wrist",wristpos);
                wristServo.setPosition(wristpos);
                
            }

            if (gamepad2.x == true) {
                double grabpos = 0.6;
                telemetry.addData("wrist",grabpos);
                grabServo.setPosition(grabpos);
                
            }
            
            if (gamepad2.y == true) {
                double grabpos = -0.6;
                telemetry.addData("wrist",grabpos);
                grabServo.setPosition(grabpos);
                
            }
            
            linearExtender.setPower(gamepad2.left_stick_y);
            telemetry.update();
        }

    }
        
        
    
    
    double get_dist(){ 
        return Math.min(scooperDistanceSensor0.getDistance(DistanceUnit.CM), scooperDistanceSensor1.getDistance(DistanceUnit.CM)); 
    }
    
    boolean checkIfBlockInScooper(){
        return scooperDistanceSensor0.getDistance(DistanceUnit.CM) < 4.2 && scooperDistanceSensor0.getDistance(DistanceUnit.CM) < 2.7;
    }
    
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
    public void autoMoveArmIfBlock(){
        double time_between_locks = 1;
            
    }
    
    public void moveLinearExtender(){
    // setTargetPosition(int position)
    // pow = (-1 to 1)
       // linearExtender.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
     
        
        
    }
    double y;
    
    public void moveFingers(){
        if (gamepad2.right_bumper){
            //fingersServo.setPosition(0);
        }
        else 
        {
          //fingersServo.setPosition(1);    
        }
    }
    public double relative_power(double intended_power)
    {
        return (13 * intended_power) / getVoltage();
    }
    
    boolean going_to_interpolate = false;
    double interpolation_amount = 0.25;
    
    public void MoveRobot(double[] direction, double rotation){
        double div_by = 1.8; 
        
        // relative power sends the amount of power that would be sent when battery is at 13V
        double wheel0 = relative_power(clamp(-direction[0] + direction[1] - rotation, -1, 1));
        double wheel1 = relative_power(clamp(direction[0] + direction[1] + rotation, -1, 1));
        double wheel2 = relative_power(clamp(-direction[0] - direction[1] + rotation, -1, 1));
        double wheel3 = relative_power(clamp(direction[0] - direction[1] - rotation, -1, 1));
        
        
        /// WIP
        /*
        if(going_to_interpolate){
            
            wheel0 = interpolation_amount * wheel0 + interpolate(omniMotor0.getPower(), wheel0, interpolation_amount);
            wheel1 = interpolation_amount * wheel1 + interpolate(omniMotor1.getPower(), wheel1, interpolation_amount);
            wheel2 = interpolation_amount * wheel2 + interpolate(omniMotor2.getPower(), wheel2, interpolation_amount);
            wheel3 = interpolation_amount * wheel3 + interpolate(omniMotor3.getPower(), wheel3, interpolation_amount);
            
        }
        */
        telemetry.addData("omniMotor0Power", omniMotor0.getPower());
        telemetry.addData("omniMotor0SetPowerTo", wheel0 / div_by);
        telemetry.addData("div_by", div_by);
        MoveMotor(omniMotor0,wheel0 / div_by);
        MoveMotor(omniMotor1,wheel1 / div_by);
        MoveMotor(omniMotor2,wheel2 / div_by);
        MoveMotor(omniMotor3,wheel3 / div_by);
    }
    
    public double interpolate(double current_value, double target_value, double amount){
        
        return (target_value - current_value) * amount;
    }
    
    public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorPowerLogs.put(motor.getDeviceName(), new ArrayList<Double>());
         return motor;
     }
     
     public void MoveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
         motor.setPower(power);
         this.motorPowerLogs.get(motor.getDeviceName()).add(power);
         
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
    // todo: write your code here

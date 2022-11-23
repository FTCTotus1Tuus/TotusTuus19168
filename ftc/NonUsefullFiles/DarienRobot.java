package org.firstinspires.ftc.NonUsefullFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Hardware;
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


@TeleOp(name="DriveOnly",group="NonCompete") 
public class DarienRobot extends LinearOpMode {
    
     // ALL VARIABLES IN THIS CHUNK ARE GLOBAL 
    //Initialize Variables
    public final double SCALE_FACTOR = 255;

    // For Movement
    // int, float, double 
    public double[] direction = {0.0,0.0};
    public double rotation;
    
    // INITIALIZATION ROBOT PARTS

    // Motor 
    public DcMotor omniMotor0; // front left
    public DcMotor omniMotor1; // front right
    public DcMotor omniMotor2; // back left
    public DcMotor omniMotor3; // back right
    
    public double match_start_time;
   
    
        // initColorSensors();
        // initColorSensors();
    
   public void runOpMode() {
            
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        
        omniMotor0.setDirection(DcMotor.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.REVERSE);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        
   
   
        //test/init telemetry
        print("Motors Init","");


        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED
        

        while (opModeIsActive()) {
            double dpadX = 0;
            double dpadY = 0;
            //calls Drive function in DarienRobot
            Drive();
        }
   }
   public void Drive()
   {
        direction[0] = Math.pow(-gamepad1.left_stick_x, 3) ;
        direction[1] = Math.pow(gamepad1.left_stick_y, 3) ;
        telemetry.addData("direction", direction);
        telemetry.update();
        rotation = Math.pow(gamepad1.right_stick_x, 3) * 0.9;
        MoveRobot(direction, rotation);
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
    
    
    public void MoveRobot(double[] direction, double rotation){
        double div_by = 1.8; 
        
        // relative power sends the amount of power that would be sent when battery is at 13V
        double wheel0 = relative_power(clamp(-direction[0] + direction[1] - rotation, -1, 1));
        double wheel1 = relative_power(clamp(direction[0] + direction[1] + rotation, -1, 1));
        double wheel2 = relative_power(clamp(-direction[0] - direction[1] + rotation, -1, 1));
        double wheel3 = relative_power(clamp(direction[0] - direction[1] - rotation, -1, 1));
        

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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         return motor;
     }
     
    //  public initColorSensors(String name) {
    //     // get a reference to the color sensor.
    //     sensorColor = hardwareMap.get(ColorSensor.class, name);
    //     sensorDistance = hardwareMap.get(DistanceSensor.class, name);

    //     // hsvValues is an array that will hold the hue, saturation, and value information.
    //     float hsvValues[] = {0F, 0F, 0F};
    //     final float values[] = hsvValues;
        
    //  //   return sensorColor, sensorDistance, hsvValues, values;
        

    //  }
     public void MoveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
         motor.setPower(power);
         
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

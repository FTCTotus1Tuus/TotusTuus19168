/*
Copyright your 
*/

package org.firstinspires.ftc.Archive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.*;

import java.util.*;
import java.io.*;

import org.firstinspires.ftc.utils.Files;
@TeleOp(name="ControllerModeB",group="NonCompete")

public class ControllerModeBackup extends LinearOpMode{
    //Initialize Variables
    Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
    Boolean replaying = false;
    
    // Our transform (Set the values to default values)
    //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d,0d,0d});
    
    // For Movement
    // int, float, double 
    double[] direction = {0.0,0.0};
    double rotation;
    
    
    // Motor 
    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right
    
    //LinearExtender
    DcMotor linearExtender;
    boolean wristCurrentPos = true;
    //Servos
    Servo wristServo;
    Servo grabServo;
    
    double targetPosition; 
    double MAX_ENCODER_POSITION = 1562;
        
    double EPSILON = 0.000000001;
    double armMotorOffset = 10;
    double currentPosition = 0;
    
    double armPower;
    
    double wristpos;
    double grabpos;
    
    public void runOpMode() {
        // connecting the motors to the config and setting to a variable name
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        
        
        
        
        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);
        linearExtender = initializeMotor("linearExtender");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        grabServo.setDirection(Servo.Direction.REVERSE);
        
        Files.test("org.firstinspires.ftc/teamcode/AutonomousData.txt", this);
        
        
        telemetry.update();
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart();
        
        while (this.opModeIsActive()) {
            // hit "X" button to break from loop and stop program
            
            direction[0] = gamepad1.left_stick_x;
            direction[1] = gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;
            
            
        
            MoveRobot(direction, -rotation);
          
            
            
            telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger);
            
            telemetry.addData("time", this.time);
            //telemetry.addData("armPowerWithChange", armPowerWithChange);
            telemetry.update();
            //MoveRobot(direction, rotation);
            
            // Move servos 
            //swings away from player
            //if (gamepad2.x && ) {
            //     if (wristCurrentPos) {
            //         wristpos = 1;
            //         telemetry.addData("wrist",wristpos);
            //         wristCurrentPos = false;
            //     }
            //     else  {
            // //swings to player
            //          wristpos = 0;
            //         telemetry.addData("wrist",wristpos);
            //         wristCurrentPos = true;
            // }
            // wristServo.setPosition(wristpos);
          //  }
            if (gamepad2.b) {
                wristpos = 1;
                telemetry.addData("wrist",wristpos);
                
            }
            //swings to player
            else if (gamepad2.x) {
                wristpos = 0;
                telemetry.addData("wrist",wristpos);
                
            } 
            //closes claw
            if (gamepad2.a) {
                grabpos = 0;
                telemetry.addData("wrist",grabpos);
                
            }
            //opens claw
            else if (gamepad2.y) {
                grabpos = 1;
                telemetry.addData("wrist",grabpos);
                
            }
                grabServo.setPosition(grabpos);
                wristServo.setPosition(wristpos);
            // if (gamepad2.a)
            // {
            //     grabpos = 1;
            //     telemetry.addData("wrist",grabpos);
            // } else {
            //     grabpos = 0;
            //     telemetry.addData("wrist",grabpos);
            // }
            // grabServo.setPosition(grabpos);
            
            linearExtender.setPower(-gamepad2.left_stick_y);
            
            
            }
        }
    
    
  
    
    
        
   
    double div_by = 2;
    public void MoveRobot(double[] direction, double rotation){
        
        double wheel0 = clamp(-direction[0] + direction[1] - rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] + rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] + rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] - rotation, -1, 1);
                
        MoveMotor(omniMotor0,wheel0/div_by);
        MoveMotor(omniMotor1,wheel1/div_by);
        MoveMotor(omniMotor2,wheel2/div_by);
        MoveMotor(omniMotor3,wheel3/div_by);
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
}

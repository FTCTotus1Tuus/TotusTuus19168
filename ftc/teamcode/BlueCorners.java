package org.firstinspires.ftc.teamcode;

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



@Autonomous(name="BlueCorners",group="Linear Opmode") 

public class BlueCorners extends DarienOpMode{

public void runOpMode() {
            
        initialize();
        


        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED

       MoveZ(-1800, 0.125);
       
        MoveY(tileDist, 0.125);
        //move to color cone
        while (omniMotor0.isBusy()){}
        //stop, add color sensing code
       //get parking space
       
 
        parkPos = getParkPos();

        Rotate(500, 0.5);
        while (omniMotor0.isBusy()){}
        //back up to pole
        MoveY(-175, 0.125);
        while (omniMotor0.isBusy()){}
        MoveY(-100, 0.11);
        while (omniMotor0.isBusy()){}
        // MoveY(5, 0.1);
        // while (omniMotor0.isBusy()){}
        sleep(100);
        //drop cone
        grabServo.setPower(0.5);
        sleep(100);
        //return
        grabServo.setPower(-0.1);
        MoveY(275, 0.125);
        while (omniMotor0.isBusy()){}
        Rotate(475, 0.5);
        
        while (omniMotor0.isBusy()){}
        //GotoParking Space
        switch(parkPos)
        {
            //Green
            case 1:
                MoveY(-600, 0.25);
                break;
            //Red
            case 2:
                MoveY(0, 0.25);
                break;
            //Blue
            case 3:
                Rotate(-25, 0.5);
                while (omniMotor0.isBusy()){}
                MoveY(625, 0.25);
                break;
        }
      while (omniMotor0.isBusy()){}
        // Rotate(950, 0.33);
        // while (omniMotor0.isBusy()){}
        // MoveY(-300, 0.15);
        wristServo.setPower(-1);
        sleep(500);
        // MoveZ(0, 0.125);
    //      MoveY(800,  0.125);
 while (opModeIsActive()){        
    
    this.telemetry.addData("TargetPos", Integer.toString(ZencoderPos));
    this.telemetry.addData("encoder", Integer.toString(linearExtender.getTargetPosition()));
    this.telemetry.addData("alpha", Integer.toString(colorSensor0.alpha()));
    this.telemetry.addData("blue: ", Integer.toString(colorSensor0.blue()));
    this.telemetry.addData("green: ", Integer.toString(colorSensor0.green()));
    this.telemetry.addData("red: ", Integer.toString(colorSensor0.red()));
    this.telemetry.addData("hue: ", Integer.toString(colorSensor0.argb()));
    this.telemetry.update();
     
    
 }   }

}

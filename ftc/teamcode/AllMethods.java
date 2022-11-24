// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.robot.Robot;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// //import com.qualcomm.robotcore.util.Hardware;
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
// @Disabled

// public class AllMethods {
//   //must be name of file
//      // ALL VARIABLES IN THIS CHUNK ARE GLOBAL 
//     //Initialize Variables
//     public final double SCALE_FACTOR = 255;

//     // For Movement
//     // int, float, double 
//     public double[] direction = {0.0,0.0};
//     public double rotation;
    
//       public int encoderPos =0;
//       public int ZencoderPos =0;
//       public double cos = Math.cos((Math.PI)/4);
//       public double constMult = (48 * 2 * (Math.PI));
//       public double constant = 537.7 / constMult;
       
//       public int tileDist = 600;
//       //600?
//       //changed to 600 to move 1 tile
//       public int armDist = 600;
//       //set arm dist to 600 as well
//     // INITIALIZATION ROBOT PARTS

//     // Motor 
//     public DcMotor omniMotor0; // front left
//     public DcMotor omniMotor1; // front right
//     public DcMotor omniMotor2; // back left
//     public DcMotor omniMotor3; // back right
    
    
//     //public DcMotor omnimotor4 = initializeMotor("linearExtender");    
//     public DcMotor linearExtender; // claw height
    
//     public CRServo grabServo; // claw open
//     public CRServo wristServo; // claw direction
    
//     public ColorSensor colorSensor0;
    
//     public double match_start_time;
    
//     public int parkPos;
   
    
//         // initColorSensors();
//         // initColorSensors();
//     public int getParkPos()
//     {
//             if (colorSensor0.red() > colorSensor0.blue() && colorSensor0.red() > (colorSensor0.green()/1.5))
//             {
//                 telemetry.addData("color", "red");
//                 telemetry.update();
//                 return 2;
//             } 
//             else if (colorSensor0.blue() > colorSensor0.red() && colorSensor0.blue() > (colorSensor0.green()/1.5))
//             {
//                 telemetry.addData("color", "blue");
//                 telemetry.update();
//                 return 3;
//             } 
//             else
//             {
//                 telemetry.addData("color", "green");
//                 telemetry.update();
//                 return 1;
//             } 
//     }
// }


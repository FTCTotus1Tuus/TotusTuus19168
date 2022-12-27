
package org.firstinspires.ftc.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

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

public class Gyro{
    
    BNO055IMU imu;
    
    Orientation angles;     
    double theta;
    double tolerance;
    
    LinearOpMode main;
    
    double EPSILON = 0.00000001;
    
    public Gyro(BNO055IMU imu){
        this.imu = imu;
    }

    public double deltaTheta(double rad){
        this.angles  = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        this.theta = (this.angles.firstAngle);
        return this.theta - rad;
    }
    
    public double getTheta(){
        return this.angles.firstAngle;
    }
    
    /*double[] joelEquation(double x, double y){
        if(y == 0 && x > 0){
            double theta_a = 0 - getTheta();
        }
        else if(y == 0 && x < 0){
            double thetha = Math.PI - getTheta();
        }
        else{
            double xSquare = Math.pow(x, 2);
            double ySquare = Math.pow(y, 2);
        
            double sqrt_x_y = Math.sqrt(xSquare + ySquare);
        
            double theta_a = Math.atan(y/(x + EPSILON)) - getTheta();
        
        }
        
        if(x > 0){
                theta_a += Math.PI;
            }
        // THETA JOYSTCIK:     x                   y
        return sqrt_x_y * Math.cos(theta_a), sqrt_x_y * Math.sin(theta_a);
        
    }*/
    
}
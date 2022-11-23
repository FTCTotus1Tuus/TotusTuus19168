package org.firstinspires.ftc.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import org.firstinspires.ftc.utils.Gyro;
import org.firstinspires.ftc.utils.Utils;

import java.lang.annotation.Target;
import com.qualcomm.robotcore.*;

import java.util.*;
import java.io.*;
import java.util.*;
import java.io.*;
public class MecanumOdometry{
    //Initialize Variables
    Gyro g; 
    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right
    
    DcMotor[] motors;
    
    PIDController pid0 = new PIDController(0.1,0,0.01);
    PIDController pid1 = new PIDController(0.1,0,0.01);
    PIDController pid2 = new PIDController(0.1,0,0.01);
    PIDController pid3 = new PIDController(0.1,0,0.01);
    
    PIDController[] pids;
    
    LinearOpMode main;
    
    static double CM_TO_PULSES = 18; //
    static double PULSES_TO_CM = 0.0555;

    MecanumOdometry(LinearOpMode _main){
        this.main = _main;
    }

    public void reset(){
        this.resetEncoderPosition(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void setMotors(DcMotor[] motors){
        this.omniMotor0 = motors[0];
        this.omniMotor1 = motors[1];
        this.omniMotor2 = motors[2];
        this.omniMotor3 = motors[3];
        
        pid0.setMotor(this.omniMotor0);
        pid1.setMotor(this.omniMotor1);        
        pid2.setMotor(this.omniMotor2);
        pid3.setMotor(this.omniMotor3);
        
        this.motors = motors;
        this.pids = new PIDController[] {pid0, pid1, pid2, pid3};
    }
    
        
  public static int[] positionToEncoder(DcMotor[] motors, double vX, double vY, double vR){
    return new int[] {-(int) (Math.round(((vX/0.908) + vY - vR) * CM_TO_PULSES)),
                                      -(int) (Math.round((-(vX/0.908) + vY - vR) * CM_TO_PULSES)),
                                      (int) (Math.round((-(vX/0.908) + vY + vR) * CM_TO_PULSES)), 
                                      (int) (Math.round(((vX/0.908) + vY + vR) * CM_TO_PULSES))};
  }
        
    public void moveToPosition(double delta_x, double delta_y, double delta_r){
        
        // Get the encoder positions we need to go to
        int encoderWheelPositions[] = positionToEncoder(this.motors, delta_x, delta_y, delta_r);
        
        // NOTE: We might have to change the direction of the power depending on direction that the motor needs to turn, IDK yet
    
        omniMotor0.setTargetPosition((encoderWheelPositions[0]));
        omniMotor1.setTargetPosition((encoderWheelPositions[1]));
        omniMotor2.setTargetPosition((encoderWheelPositions[2]));
        omniMotor3.setTargetPosition((encoderWheelPositions[3]));
    
        double TOLERANCE = 0.01;
        
        // (target - current) 
        
        // (Math.abs(encoderWheelPositions[0] - omniMotor0.getCurrentPosition()) > Math.abs(encoderWheelPositions[0]) * TOLERANCE)
        // (Math.abs(encoderWheelPositions[1] - omniMotor1.getCurrentPosition()) > Math.abs(encoderWheelPositions[1]) * TOLERANCE)
        // (Math.abs(encoderWheelPositions[2] - omniMotor2.getCurrentPosition()) > Math.abs(encoderWheelPositions[2]) * TOLERANCE)
        // (Math.abs(encoderWheelPositions[3] - omniMotor3.getCurrentPosition()) > Math.abs(encoderWheelPositions[3]) * TOLERANCE)
        // 
        
        double power = 0.4;
        
        while((Math.abs(encoderWheelPositions[0] - omniMotor0.getCurrentPosition()) > Math.abs(encoderWheelPositions[0]) * TOLERANCE) ||
              (Math.abs(encoderWheelPositions[1] - omniMotor1.getCurrentPosition()) > Math.abs(encoderWheelPositions[1]) * TOLERANCE) || 
              (Math.abs(encoderWheelPositions[2] - omniMotor2.getCurrentPosition()) > Math.abs(encoderWheelPositions[2]) * TOLERANCE) ||
              (Math.abs(encoderWheelPositions[3] - omniMotor3.getCurrentPosition()) > Math.abs(encoderWheelPositions[3]) * TOLERANCE)){
            if((Math.abs(encoderWheelPositions[0] - omniMotor0.getCurrentPosition()) > Math.abs(encoderWheelPositions[0]) * TOLERANCE))
                //omniMotor0.setPower(0.5);
                omniMotor0.setPower(_sign(encoderWheelPositions[0] - omniMotor0.getCurrentPosition()) * power);
            else{
                omniMotor0.setPower(0);
            }
            if((Math.abs(encoderWheelPositions[1] - omniMotor1.getCurrentPosition()) > Math.abs(encoderWheelPositions[1]) * TOLERANCE))
                //omniMotor1.setPower(0.5);
                omniMotor1.setPower(_sign(encoderWheelPositions[1] - omniMotor1.getCurrentPosition()) * power);
            else{
                omniMotor1.setPower(0);
            }
            if((Math.abs(encoderWheelPositions[2] - omniMotor2.getCurrentPosition()) > Math.abs(encoderWheelPositions[2]) * TOLERANCE))
                //omniMotor2.setPower(0.5);
                omniMotor2.setPower(_sign(encoderWheelPositions[2] - omniMotor2.getCurrentPosition()) * power);
            else{
                omniMotor2.setPower(0);
            }
            if((Math.abs(encoderWheelPositions[3] - omniMotor3.getCurrentPosition()) > Math.abs(encoderWheelPositions[3]) * TOLERANCE))
                //omniMotor3.setPower(0.5);
                omniMotor3.setPower(_sign(encoderWheelPositions[3] - omniMotor3.getCurrentPosition()) * power);
            else{
                omniMotor3.setPower(0);
            }
        }
        
        
        double current_time = this.main.time;
    /*    
        while(!(pid0.isSettled() && pid1.isSettled() && pid2.isSettled() && pid3.isSettled())){
            pid0.controlMotor((double) encoderWheelPositions[0], current_time);
            pid1.controlMotor((double) encoderWheelPositions[1], current_time);
            pid2.controlMotor((double) encoderWheelPositions[2], current_time);
            pid3.controlMotor((double) encoderWheelPositions[3], current_time);
            this.main.telemetry.addLine(this.summary());
            
            this.main.telemetry.update();
            // FIX THIS SO THAT IT IS THE ACTUAL TIME
            current_time = this.main.time;
        }
        */
        
        omniMotor0.setPower(0);
        omniMotor1.setPower(0);
        omniMotor2.setPower(0);
        omniMotor3.setPower(0);
        
    }
    /*
    public double[] positionOfRobot(){    
        
        double wheel0 = relative_power(clamp(-direction[0] - direction[1] - rotation, -1, 1));
        double wheel1 = relative_power(clamp(direction[0] - direction[1] + rotation, -1, 1));
        double wheel2 = relative_power(clamp(-direction[0] + direction[1] + rotation, -1, 1));
        double wheel3 = relative_power(clamp(direction[0] + direction[1] - rotation, -1, 1));
        
        double x = (PULSES_TO_CM) * (-omniMotor0.getCurrentPosition() + omniMotor1.getCurrentPosition() - omniMotor2.getCurrentPosition() - omniMotor3.getCurrentPosition());
        double y = (PULSES_TO_CM) * (-omniMotor0.getCurrentPosition() - omniMotor1.getCurrentPosition() + omniMotor2.getCurrentPosition() + omniMotor3.getCurrentPosition());
        
        return new double[] {x, y};
    }*/
    
    public int _sign(int a){
        if(a == 0){
            return 1;
        }
        return a / Math.abs(a);
    }
    
    public void resetEncoderPosition(DcMotor.RunMode newMode){
        
        for(DcMotor _motor : this.motors){
            _motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for(DcMotor _motor : this.motors){
            _motor.setTargetPosition(0);
        }
        for(DcMotor _motor : this.motors){
            _motor.setMode(newMode);
        }
    }
    
    boolean isSettled(double errors[], double delta_t){
        double THRESHOLD_FOR_SETTLED_SLOPE = 0.001;
        
        if(getSlopeBetweenTwoPoints(errors[0], errors[1], delta_t) <= THRESHOLD_FOR_SETTLED_SLOPE){
            return true;
        }
        
        return false;
    }
    
    double getSlopeBetweenTwoPoints(double point0, double point1, double delta_t){
        return (point1 - point0) / delta_t;
    }

    /*
    double[] positionFunction(DcMotor[] motors){
        double theta = 0;
        
        double scalar_value = 112/(4000*Math.sqrt(2));
        
        double P0 = omniMotor0.getCurrentPosition();
        double P1 = omniMotor1.getCurrentPosition();
        double P2 = omniMotor2.getCurrentPosition();
        double P3 = omniMotor3.getCurrentPosition();
        
        
        double x = (scalar_value) * ((Math.cos(PI/4 + theta) * (P0 + P3) + Math.cos(3 * PI/4 + theta) * (P1 + P2)));
        double y = (scalar_value) * ((Math.sin(PI/4 + theta) * (P0 + P3) + Math.sin(3 * PI/4 + theta) * (P1 + P2)));
        
        return new double[] {x,y};
    }*/
     public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
     }
     
     public String summary(){
         int i = 0;
         String s = "";
         for(DcMotor _motor : this.motors){
             s += "motor" + Integer.toString(i) + ":\n\tposition: " + Integer.toString(_motor.getCurrentPosition()) + " \tpower:" + Double.toString(Utils.roundSigFigs(_motor.getPower(), 3));
             s += "\npid:" + pids[i].getValues() + "\n";
             i++;
         }
         return s;
     }
     
     public String positionsToString(){
         return Integer.toString(omniMotor0.getCurrentPosition()) + ", " + Integer.toString(omniMotor1.getCurrentPosition()) + ", " + Integer.toString(omniMotor2.getCurrentPosition()) + ", " + Integer.toString(omniMotor3.getCurrentPosition());
     }
    
}

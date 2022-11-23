package org.firstinspires.ftc.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.utils.Utils;

import java.util.*;
import java.io.*;


import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDController{
    double kP;
    double kI;
    double kD;
    
    DcMotor motor;
    
    double previous_error = 1;
    double error = 1;
    double total_error = 0;
    
    double power = 0;
    double delta_t = 0;
    
    double previous_time = -1;
    
    double EPSILON = 0.000000000000000001;
    
    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    public double calculateError(double current, double target){
        return (target-current) / target;
    }
    
    public void controlMotor(double target_position, double current_time){
        
        boolean use_proportional = true;
        boolean use_integral = true;
        boolean use_derivative = true;
        
        this.error = calculateError((double) motor.getCurrentPosition(), (double) target_position);
        
        if(!(previous_time == -1)){
            this.delta_t = current_time - this.previous_time;
            this.total_error += error * delta_t;
        }            
        else{
            use_integral = false;
            use_derivative = false;
        }
        
        if(use_proportional)
            power += getProportional() * this.kP;
        if(use_integral){
            power += getIntegral() * this.kI;
        }
        if(use_derivative){
            power += getDerivative() * this.kD;
        }
        
        // MAKE SURE TO ADD RELATIVE PWOER
        motor.setPower(power);
        
        previous_time = current_time;
        previous_error = error;
        
    }
    
    public double getProportional(){
        return error;
    }
    
    public double getIntegral(){
        return total_error;
    }
    
    public double getDerivative(){
        return (error - previous_error) / delta_t;
    }
    
    public boolean isSettled(){
        double THRESHOLD_FOR_SETTLED_SLOPE = 0.001;
        double THRESHOLD_FOR_ERROR = 0.01;
        
        if(getSlopeBetweenTwoPoints(previous_error, error, delta_t) <= THRESHOLD_FOR_SETTLED_SLOPE && Math.abs(error) < THRESHOLD_FOR_ERROR){
            return true;
        }
        
        return false;
    
    }
    
    public void setMotor(DcMotor motor){
        this.motor = motor;
    }
    
    public void reset(){
        this.previous_time = -1;
        this.previous_error = 1;
        this.total_error = 0;
        this.error = 1;
        this.delta_t = 0;
        this.power = 0;
    }
    
    
    public String getValues(){
        return Double.toString(Utils.roundSigFigs(this.getProportional(), 3)) + ", " + Double.toString(Utils.roundSigFigs(this.getIntegral(), 3)) + ", " + Double.toString(Utils.roundSigFigs(this.getDerivative(), 3));
    }

    double getSlopeBetweenTwoPoints(double point0, double point1, double delta_t){
        return (point1 - point0) / (delta_t + EPSILON);
    }
};
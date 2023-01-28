package org.firstinspires.ftc.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
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

import org.firstinspires.ftc.utils.Motor;

public class Wheels {
    
    double wheel0Power;
    double wheel1Power;
    double wheel2Power;
    double wheel3Power;
    
    
    Hashtable<String, Motor> motors = new Hashtable<String, Motor>();
    
    Wheels(LinearOpMode linearOpMode){
        motors.put("omniMotor0", new Motor("omniMotor0", linearOpMode, true));
        motors.put("omniMotor1", new Motor("omniMotor1", linearOpMode, true));
        motors.put("omniMotor2", new Motor("omniMotor2", linearOpMode, true));
        motors.put("omniMotor3", new Motor("omniMotor3", linearOpMode, true));
        
        motors.get("omniMotor1").motor.setDirection(DcMotor.Direction.REVERSE);
        motors.get("omniMotor2").motor.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void move(double[] direction, double rotation){
        
        wheel0Power = Utils.clamp(-direction[0] + direction[1] - rotation, -1, 1);
        wheel1Power = Utils.clamp(direction[0] + direction[1] + rotation, -1, 1);
        wheel2Power = Utils.clamp(-direction[0] + -direction[1] + rotation, -1, 1);
        wheel3Power = Utils.clamp(direction[0] + -direction[1] - rotation, -1, 1);
                
        moveMotor(this.motors.get("omniMotor0").motor,wheel0Power);
        moveMotor(this.motors.get("omniMotor1").motor,wheel1Power);
        moveMotor(this.motors.get("omniMotor2").motor,wheel2Power);
        moveMotor(this.motors.get("omniMotor3").motor,wheel3Power);
    }
    
    public void moveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
         motor.setPower(power);
     }
    
    public String toString(){
        return "WHEELS:\n\t1:\t";
    }
        
    // todo: write your code here
}
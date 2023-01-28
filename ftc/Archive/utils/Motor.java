package org.firstinspires.ftc.utils;

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
import org.firstinspires.ftc.utils.*;

public class Motor{
    
    public DcMotor motor;
    
    public int encoderPosition;
    int encoderPositionOffset;
    
    Motor(String name, LinearOpMode linearOpMode, boolean resetEncoderPositionOnStartUp){
        this.motor = linearOpMode.hardwareMap.get(DcMotor.class, name);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(resetEncoderPositionOnStartUp){
            this.encoderPositionOffset = this.motor.getCurrentPosition();
        }
    }
    
    public void setPower(double power){
        this.motor.setPower(power);
    }
    
    public void update(){
        this.encoderPosition = this.motor.getCurrentPosition() - this.encoderPositionOffset;
    }
    
    public int getPosition(){
        this.encoderPosition = this.motor.getCurrentPosition() - this.encoderPositionOffset;
        return encoderPosition;
    }
    
}

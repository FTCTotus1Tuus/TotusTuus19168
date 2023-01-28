package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Meet3BlueCorners",group="NonCompete") 

public class Meet3BlueCorners extends DarienOpMode{

    public void runOpMode() {
            
        initialize();
        
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED
        grabServo.setPower(0.55); //closes the grab servo as a safety measure
        MoveZ(-5400, armPower); // moves the arm up
        //Moves center of the robot to the center of the first tile
        MoveY(robotCenterAtStart + 2*tileDist + 2*tileDist/5 - 20, autoPower); 
        //8750 is the ratio for how long you have to wait to detect color on any given power
        sleep(2200);
        grabServo.setPower(0.1); // stop squeezing the claw
        parkPos = getParkPos(); //reads signal cone
        waitForMotors();
        MoveY(-2*tileDist/5 + 90, autoPower); //backs up to center of high pole tile
        waitForMotors();
        
        //Start loop to stack cones on high
        for (int i=0; i<conesMax; i++) {
            dropBlueCone();
            
            Rotate(270); //turn towards stack
            waitForMotors();
            MoveZ(-570 + (i*165), armPower); //lower arm
            moveToConeStack();
        //finished grabbing cone. Placing on high pole
        }
        dropBlueCone();
        //parks
        Rotate(270);
        while(omniMotor0.isBusy()){}
        
        switch(parkPos)
        {
            //Green
            case 1:
                MoveY(-615, 0.5);
                break;
            //Red
            case 2:
                MoveY(0, 0.5);
                break;
            //Blue
            case 3:
                MoveY(615, 0.5);
                break;
        }
        waitForMotors();
        
        while (opModeIsActive()){        
    
            this.telemetry.addData("TargetPos", Integer.toString(ZencoderPos));
            this.telemetry.addData("encoder", Integer.toString(linearExtender.getCurrentPosition()));
            this.telemetry.addData("alpha", Integer.toString(colorSensor0.alpha()));
            this.telemetry.addData("blue: ", Integer.toString(colorSensor0.blue()));
            this.telemetry.addData("green: ", Integer.toString(colorSensor0.green()));
            this.telemetry.addData("red: ", Integer.toString(colorSensor0.red()));
            this.telemetry.addData("hue: ", Integer.toString(colorSensor0.argb()));
            this.telemetry.addData("Rotation: ", Double.toString(getRawHeading()));
            this.telemetry.update();
 
        }   
    }
    private void dropBlueCone() {
        //starts at the center of the high pole tile drops the cone on that pole.
        Rotate(225); //turn towards high pole
        waitForMotors();
        MoveY(-455, autoPower - 0.03); //push towards pole
        waitForMotors();
        MoveY(17, 0.1);
        waitForMotors();
        sleep(300);
        MoveZ(-3000, armPower); //lower linear extender
        while(linearExtender.isBusy()){}
        grabServo.setPower(-0.55); //open claw to drop cone
        sleep(50);
        Rotate(225); //re-allign robot
        waitForMotors();
        MoveY(235, autoPower); //back up to center of tile
        sleep(125);
        grabServo.setPower(0); //stop opening claw
        waitForMotors();
    }

}

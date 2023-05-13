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

@Autonomous(name="StateRedCorners",group="Linear Opmode") 

public class StateRedCorners extends DarienOpMode{

    public void runOpMode() {
            
        initialize();
        
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED
        
        grabServo.setPower(1); //closes the grab servo as a safety measure
        MoveZ(-3150, armPower-.25); // moves the arm up
        wristServo.setPower(0.5); // moves wrist towards pole
        //Moves center of the robot to the center of the first tile
        MoveY(robotCenterAtStart + tileDist + 2*tileDist/7, autoPower*1.4); 
        //8750 is the ratio for how long you have to wait to detect color on any given power
        sleep(1500);
        wristServo.setPower(0.1);
        // grabServo.setPower(0.1); // stop squeezing the claw
        // parkPos = getParkPos(); //reads signal cone
        waitForMotors();
        // MoveY(-2*tileDist/5 + 40 , autoPower); //backs up to center of high pole tile
        MoveY(-2*tileDist/7 + 25, autoPower*1.3); //backs up to center of mid pole tile
        waitForMotors();
        // dropRedCone();
        Rotate(270, -1);
        waitForMotors();
        MoveX(tileDist+25, autoPower*1.2);
        waitForMotors();
        Rotate(270,0);
        // moveToConeStack();
        //Start loop to stack cones on high
        for (int i=0; i<conesMax; i++) {
            print("loop", "");
            grabServo.setPower(1); // close grabber with gusto
            sleep(500);
            MoveZ(-1600, armPower); //move Linear Extender up
            sleep(500);
            
            MoveY((robotCenterAtStart+35), autoPower); //move away from conestack
            wristServo.setPower(1);
            waitForMotors();
            
            MoveX(-tileDist/2-15, autoPower);// move sideways
            waitForMotors();
            Rotate(270,0);// realign
            waitForMotors();
            MoveY(125, autoPower);// go towards low pole
            waitForMotors();
            print("here", "");
            
            Rotate(270,0);// re-allign
            waitForMotors();
            MoveZ(-1150, armPower); // lower arm to low pole
            sleep(300);
            grabServo.setPower(-1);// drop cone
            Rotate(270,0);
            waitForMotors();// re-allign
            MoveZ(-1600, armPower);
            sleep(300);
            grabServo.setPower(-0.2);
            // MoveZ(-1500, 1); //raise to clear the low pole 
            sleep(100);
            wristServo.setPower(-1); //swing claw back to conestack
            sleep(100);
            MoveY(-95,autoPower);//move back from pole
            waitForMotors();
            MoveZ(-530 + ((i+1)*165), armPower-0.1);//lower arm
            

            
            if (i<(conesMax-1)) {
            
            MoveX(tileDist/2, autoPower); // move back to conestack
            waitForMotors();
            Rotate(270, 0); //realign
            waitForMotors();
            MoveY(-robotCenterAtStart-60, autoPower); //move into the conestack
            waitForMotors(); }
            
            else {
                MoveX(-tileDist/2, autoPower); //go to open square near the triangle
                waitForMotors();
                Rotate(270, 0);
                waitForMotors();
            }
        //finished grabbing cone. Placing on high pole
        }
        
        //parks
        // Rotate(90);
        
        switch(parkPos)
        {
            //Blue
            case 3:
                MoveY(2*tileDist-50, 0.3);
                break;
            //Red
            case 2:
                MoveY(tileDist-50, 0.3);
                break;
            //Green
            case 1:
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
    private void dropRedCone() {
        //starts at the center of the high pole tile drops the cone on that pole.
        Rotate(315,-1); //turn towards high pole
        waitForMotors();
        MoveY(360, autoPower - 0.03); //push towards pole
        waitForMotors();
        sleep(100);
        MoveY(-17, 0.1); // move slightly from the pole
        waitForMotors();
        sleep(300);
        MoveZ(-2000, armPower); //lower linear extender
        while(linearExtender.isBusy()){}
        grabServo.setPower(-0.55); //open claw to drop cone
        sleep(50);
        Rotate(315, 0); //re-allign robot
        waitForMotors();
        MoveY(-283, autoPower); //back up to center of tile
        sleep(125);
        grabServo.setPower(0); //stop opening claw
        waitForMotors();
    }

}

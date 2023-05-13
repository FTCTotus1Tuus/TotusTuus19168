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

@Autonomous(name="OffseasonBlueCorners",group="Linear Opmode") 

public class OffseasonBlueCorners extends DarienOpMode{
    public double[] offset = {-1617.0,0.0};
    public double rotation = 270;
    
    public void runOpMode() {
        initialize();
        
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart(); // WAITS UNTIL START BUTTON IS PRESSED

        
        MoveXandY(tileDist, tileDist, autoPower);
        
        while (opModeIsActive()){        
    
            this.telemetry.addData("TargetPos", Integer.toString(ZencoderPos));
            this.telemetry.addData("encoder", Integer.toString(linearExtender.getCurrentPosition()));
            this.telemetry.addData("alpha", Integer.toString(colorSensor0.alpha()));
            this.telemetry.addData("blue: ", Integer.toString(colorSensor0.blue()));
            this.telemetry.addData("green: ", Integer.toString(colorSensor0.green()));
            this.telemetry.addData("red: ", Integer.toString(colorSensor0.red()));
            this.telemetry.addData("hue: ", Integer.toString(colorSensor0.argb()));
            this.telemetry.addData("Rotation: ", Double.toString(getRawHeading()));
            this.telemetry.addData("encoder 0", Integer.toString(omniMotor0.getCurrentPosition()));
            this.telemetry.addData("encoder 1", Integer.toString(omniMotor1.getCurrentPosition()));
            this.telemetry.addData("encoder 2", Integer.toString(omniMotor2.getCurrentPosition()));
            this.telemetry.addData("encoder 3", Integer.toString(omniMotor3.getCurrentPosition()));
            this.telemetry.addData("rot amount 0", Integer.toString(rotAmounts0));
            this.telemetry.addData("rot amount 1", Integer.toString(rotAmounts1));
            this.telemetry.addData("rot amount 2", Integer.toString(rotAmounts2));
            this.telemetry.addData("rot amount 3", Integer.toString(rotAmounts3));
            this.telemetry.addData("Coords x", Double.toString(getCoords()[0]));
            this.telemetry.addData("Coords y", Double.toString(getCoords()[1]));
            this.telemetry.update();
 
        }   


}}

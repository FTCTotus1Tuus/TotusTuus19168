package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// @Autonomous

public class DarienOpModeTest extends LinearOpMode{
              // ^ must be name of file
     
    //Initialize Variables

    // For Movement
    // int, float, double 
    public double[] direction = {0.0,0.0};
    public int ZencoderPos =0;
    public double rotation;
    public double[] offset = {0.0, 0.0};
    
        // ODOMETRY
            public double cos = Math.cos((Math.PI)/4);
            public double constMult = (48 * 2 * (Math.PI));
            public double constant = 537.7 / constMult;
    
    
    // GYRO ROTATION
        private BNO055IMU       imu;  
        public int encoderPos0,encoderPos1,encoderPos2,encoderPos3 =0;
        public double rotateDirection;
    
    
    // MOVEMENT CONSTANTS
        public int tileDist = 600;
        public int robotLength = 420;
        public int robotCenterAtStart = tileDist/2 - robotLength/2; //Distance to the center of the first tile at start
        public double autoPower = .15;
        public double armPower = .45;
        public double autoAcc =0.002;
        public int conesMax = 1;
        public double accIncrement;
        public double constantPowerTime;
        public double targetPower;
        public double acc;
        public int encoderPos;
        public double initialPower = 0.01;
        public boolean interrupt = true;
    
    // INITIALIZATION OF ROBOT PARTS

        // WHEEL MOTORS 
            public DcMotor omniMotor0; // front left
            public DcMotor omniMotor1; // front right
            public DcMotor omniMotor2; // back left
            public DcMotor omniMotor3; // back right
    
        // ARM/CLAW MOTORS
            public DcMotor linearExtender; // claw height
            public CRServo grabServo; // claw open
            public CRServo wristServo; // claw direction
    
    
    // PARKING
    
        public ColorSensor colorSensor0;
        public int parkPos;
        public ElapsedTime colorTime =  new ElapsedTime();
   
   public void initialize() {
       
       
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        linearExtender = initializeMotor("linearExtender");
        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        grabServo = hardwareMap.get(CRServo.class, "grabServo");
        wristServo = hardwareMap.get(CRServo.class, "wristServo");
        grabServo.setDirection(CRServo.Direction.REVERSE);
        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        linearExtender.setDirection(DcMotor.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //servo0 = initializeServo("servo0");
        //servo0 = hardwareMap.get(Servo.class, "wristServo");
        //grabServo = hardwareMap.get(Servo.class, "grabServo");
   
        //test/init telemetry
        print("Motors Init","");
   }    
    public void runOpMode(){
        //Necessary emptpy method for linear op mode
    }
    
    public double varPower()
    {
        double appliedPower;
        accIncrement = targetPower/acc;
        constantPowerTime = encoderPos - 2*accIncrement;
        if (omniMotor0.getCurrentPosition() < accIncrement)
        {
            appliedPower = omniMotor0.getCurrentPosition()*acc + initialPower;
        }
        else if (omniMotor0.getCurrentPosition() >= accIncrement && omniMotor0.getCurrentPosition() <= accIncrement + constantPowerTime)
        {
            appliedPower = targetPower;
        }
        else if (omniMotor0.getCurrentPosition() > accIncrement + constantPowerTime && omniMotor0.getCurrentPosition() < encoderPos)
        {
            appliedPower = (encoderPos-omniMotor0.getCurrentPosition())*acc;
        }
        
        else
        {
            appliedPower = 0;
        }
        
        return appliedPower;
    }
    public double varPowerRot(double heading, double acc, double targetPower)
        {
        double appliedPower;
        accIncrement = targetPower/acc;
        constantPowerTime = heading - 2*accIncrement;
        if (getRawHeading() < accIncrement)
        {
            appliedPower = getRawHeading()*acc + initialPower;
        }
        else if (getRawHeading() >= accIncrement && getRawHeading() <= accIncrement + constantPowerTime)
        {
            appliedPower = targetPower;
        }
        else if (getRawHeading() >= heading)
        {
            appliedPower = 0;
        }
        else
        {
            appliedPower = (heading-getRawHeading())*acc;
        }
        return appliedPower;
        }
    public int getParkPos()
    {
            if (colorSensor0.red() > colorSensor0.blue() && colorSensor0.red() > (colorSensor0.green()/1.2))
            {
                telemetry.addData("color", "red");
                telemetry.update();
                return 2;
            } 
            else if (colorSensor0.blue() > colorSensor0.red() && colorSensor0.blue() > (colorSensor0.green()/1.2))
            {
                telemetry.addData("color", "blue");
                telemetry.update();
                return 3;
            } 
            else
            {
                telemetry.addData("color", "green");
                telemetry.update();
                return 1;
            } 
    }
    public void moveToConeStack(){
        // MoveY(250,autoPower);
        // sleep(100);
        // while(omniMotor0.isBusy()){}
        // Rotate(270);
        // while(omniMotor0.isBusy()){}
        // sleep(100);
        MoveY(tileDist + robotCenterAtStart,autoPower, autoAcc); // robot to conestack
        wristServo.setPower(-1); // wrist towards conestack
        sleep(750);
        wristServo.setPower(0); //turn off wrist servo
        updateMotors();
        grabServo.setPower(-1); // close grabber with gusto
        sleep(750);
        grabServo.setPower(0); // stop closing grabber
        MoveZ(-5400, armPower); //move Linear Extender up
        sleep(500);
        
        MoveY(-(tileDist + robotCenterAtStart), autoPower, autoAcc); //move away from conestack
        wristServo.setPower(1);
        sleep(750);
        updateMotors();
    }
    public void RotateOld(int rotation, double power)
    {
        encoderPos = rotation;
        setTargetPosRot();
        setRunMode();
        setPower(power);
    }
    // maybe change V to Z?
    // yessir        /
    //              \/
     public void Rotate(double heading, double acc, double targetPower)
    {   
    boolean isRotating = true;
    setToRotateRunMode();
    setRotatePower(varPowerRot(heading, 0.01, 0.75), rotateDirection);
    if (getRawHeading() - heading > 0){
        rotateDirection = 1;
    telemetry.addData("starting rotate function", "");
    telemetry.update();
    }
    else {
        rotateDirection = -1;
    telemetry.addData("starting rotate function", "");
    telemetry.update();
    }
    while (isRotating){
        
        
        if (Math.abs(getRawHeading() - heading) <= 1) {
            // resetTargetRotPos();
            setPower(0);
            isRotating = false;
        telemetry.addData("finised function 1", "");
        telemetry.update();    
            
        }
    }
        telemetry.addData("finised function", "");
        telemetry.update();    
        
    }
    // maybe change V to Z?
    // yessir        /
    //              \/
    public void MoveZ(int z, double power){
       
       linearExtender.setTargetPosition(z);
       linearExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       linearExtender.setPower(power);
       
   }
   
   public void MoveY(int y, double power, double acceleration){
       
       targetPower = power;
       acc = acceleration;
       encoderPos = (int) Math.floor((y * constant+ 0.5));
       setTargetPosY();
       
       setRunMode();
       
       setPower(varPower());
       colorTime.reset();
   }
   
   
      public void MoveX(int x, double power, double acceleration){
       targetPower = power;
       acc = acceleration;
       encoderPos = (int) Math.floor((x * constant) + 0.5);
       setTargetPosX();
       
       setRunMode();
       
       setPower(varPower());
       colorTime.reset();
       //power = 0;
       //setPower(power);
   }
   
   
   public void setTargetPosY()
   {
        
       omniMotor0.setTargetPosition(encoderPos);
       omniMotor1.setTargetPosition(encoderPos);
       omniMotor2.setTargetPosition(encoderPos);
       omniMotor3.setTargetPosition(encoderPos);
       
   }
   
      public void setTargetPosX()
   {
        
       omniMotor0.setTargetPosition(-encoderPos);
       omniMotor1.setTargetPosition(encoderPos);
       omniMotor2.setTargetPosition(encoderPos);
       omniMotor3.setTargetPosition(-encoderPos);
       
   }
    public void setTargetPosRot()
    {
       omniMotor0.setTargetPosition(encoderPos);
       omniMotor1.setTargetPosition(-encoderPos);
       omniMotor2.setTargetPosition(encoderPos);
       omniMotor3.setTargetPosition(-encoderPos);
    }
    public void resetTargetRotPos() {
            encoderPos0 = omniMotor0.getCurrentPosition();
            encoderPos1 = omniMotor1.getCurrentPosition();
            encoderPos2 = omniMotor2.getCurrentPosition();
            encoderPos3 = omniMotor3.getCurrentPosition();
            setRunMode();
            omniMotor0.setTargetPosition(encoderPos0);
            omniMotor1.setTargetPosition(encoderPos1);
            omniMotor2.setTargetPosition(encoderPos2);
            omniMotor3.setTargetPosition(encoderPos3);
    }
   

   
    public void setRunMode()
   {
        omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

   }
   
    public void setToRotateRunMode(){
        omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
   public void setPower(double power)
   {
       omniMotor0.setPower(power);
       omniMotor1.setPower(power);
       omniMotor2.setPower(power);
       omniMotor3.setPower(power);
       
   }
   
   public void setRotatePower(double power, double direction){
        omniMotor0.setPower(direction*power);
        omniMotor1.setPower(-direction*power);
        omniMotor2.setPower(direction*power);
        omniMotor3.setPower(-direction*power);
        
    }

    

    
    public void updateMotors()
    {
        while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy())
        {
            setPower(varPower());
            this.telemetry.addData("Power: ", Double.toString(varPower()));
            this.telemetry.addData("encoder", Integer.toString(omniMotor0.getCurrentPosition()));
            this.telemetry.addData("encoder", Integer.toString(encoderPos));
            this.telemetry.addData("accIncrement", Double.toString(accIncrement));
            this.telemetry.addData("accIncrement", Double.toString(targetPower));
            this.telemetry.update();

        }
    }
    public void updateMotorsColorDetect()
    {
        while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy())
        {
            setPower(varPower());
            if (colorTime.milliseconds() > 2200 && colorTime.milliseconds() < 2400)
            {
                parkPos = getParkPos();
            }
        }
    }
   
   
//   public void controllerTest()
//   {
//           if(gamepad1.y){
//         this.telemetry.addData("Moving", "North...");
//         MoveY(1 * tileDist,  0.125);
//     };
//     if(gamepad1.a){
//         this.telemetry.addData("Moving", "South...");
//         MoveY(-1 * tileDist,  0.125);
//     }
//     if(gamepad1.b){
//         this.telemetry.addData("Moving", "West...");
//         MoveX(1 * tileDist,  0.125);
//     };
//     if(gamepad1.x){
//         this.telemetry.addData("Moving", "East...");
//         MoveX(-1 * tileDist,  0.125);
//     }
    
//     if(gamepad1.dpad_down){
//         this.telemetry.addData("Moving", "Down...");
//         MoveZ(1 * armDist,  0.125);
//     };
//     if(gamepad1.dpad_up){
//         this.telemetry.addData("Moving", "Up...");
//         MoveZ(-1 * armDist,  0.125);
//     }
    
//     if(gamepad1.dpad_left){
//         this.telemetry.addData("Moving Claw", "open");
//         wristServo.setPosition(-10);
//     };
//     if(gamepad1.dpad_right){
//         this.telemetry.addData("Moving Claw", "close");
//         wristServo.setPosition(10);
//     }
//     if(gamepad1.left_bumper){
//         this.telemetry.addData("Moving Claw", "L");
//         grabServo.setPosition(-10);
//     };
//     if(gamepad1.right_bumper){
//         this.telemetry.addData("Moving Claw", "R");
//         grabServo.setPosition(10);
//     }
//   }
   
    int move_to_position;
    double y;
    
    public void print(String Name, String message)
    {
        telemetry.addData(Name, message);
        telemetry.update();
    }

    public double relative_power(double intended_power)
    {
        return (13 * intended_power) / 13; //getVoltage()
    }
    
    boolean going_to_interpolate = false;
    double interpolation_amount = 0.25;
    
    
   public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         return motor;
     }
     
    public Servo initializeServo(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        Servo motor = hardwareMap.get(Servo.class, name);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         return motor;
     }

     public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
     }
     public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
        }
        
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle >= 0){
            return angles.firstAngle;
       
        }
        else {
            return angles.firstAngle + 361;
        }
    }
        
    boolean timeBetween(double startTime, double endTime){
        if((this.time  >= startTime) && (this.time <= endTime)){
            return true;
        }
        return false; 
    }
    public void interrupt()
    {
        while (interrupt = true){}   
    }
      
}
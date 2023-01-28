/*
Copyright your 
*/

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
//@TeleOp(name="CalibrateVelocity",group="Linear Opmode")

public class CalibrateVelocity extends LinearOpMode{
    //Initialize Variables
    Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
    Boolean replaying = false;
    
    // Our transform (Set the values to default values)
    //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d,0d,0d});
    
    // For Movement
    // int, float, double 
    double[] direction = {0.0,0.0};
    double rotation;
    
    // INITIALIZATION
    
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    
    // Motor 
    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right
    
    DcMotor armMotor; // motor arm
    DcMotor wheelMotor; // motor for da wheel
    
    DcMotor hdHexMotor0; // arm control 
    
    //Servo wristServo; // wrist movement - change to wrist servo
    //Servo fingersServo; // fingers for grabbing - change to finger servo
    CRServo armSpin; // CR Servo
    
    double targetPosition; // 
    double MAX_ENCODER_POSITION = 8700;
    
    double EPSILON = 0.000000001;
    double armMotorOffset = 10;
    double currentPosition = 0;
    
    double armPower;
    
    public void runOpMode() {
        // connecting the motors to the config and setting to a variable name
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        armMotor = initializeMotor("armMotor");
        wheelMotor = initializeMotor("wheelMotor");
        //wristServo = hardwareMap.get(Servo.class, "wristServo");
        //fingersServo = hardwareMap.get(Servo.class, "fingersServo");
        armSpin = hardwareMap.get(CRServo.class, "armSpin");
        
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        omniMotor0.setDirection(DcMotor.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.REVERSE);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        
        
        Files.test("org.firstinspires.ftc/teamcode/AutonomousData.txt", this);
        
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Initialize each dictionary
        // arr = {0,7,8}
        // arr[1] = 2;
        // print (arr) -- {028}
        
        //telemetry.addLine("asd");
        telemetry.update();
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart();
        
        targetPosition = ((((double) armMotor.getCurrentPosition()))) * -1 + armMotorOffset;
        armSpin.setDirection(DcMotor.Direction.FORWARD);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        // CHANGE THE POWER WE RUNNING AT
        double power = -0.25;
        
        // CHANGE THE TIME WE RUN FOR
        double run_for = 2;
        
        // CHANGE THE AXIS WE TRAVELING IN
        int axis_we_traveling = 0;
        
        Boolean player_stopped = false;
        Boolean turned_off = false;
        
        double time_offset = this.time;
        
        while (this.opModeIsActive()) {
            
            if( ! (player_stopped) && (gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b)){
                turned_off = true;
            }
            else{
                if((this.time - time_offset) >= run_for){
                    turned_off = true;
                }
            }
            
            
            // If the player will be stopping the robot manually
            if( ! turned_off){
                
                // If the robot isn't turned off
                if( ! player_stopped){
                    direction[axis_we_traveling] = power;
                }
                else{
                    direction[axis_we_traveling] = power;
                    
                }
            }
            
            else{
                    direction[axis_we_traveling] = 0;
            }
            MoveRobot(direction, 0);
            telemetry.addLine()
                .addData("RUNNING FOR: ", run_for)
                .addData("POWER SET TO: ", power)
                .addData("AXIS (x - 0; y - 1)", axis_we_traveling);
            telemetry.addLine()
                .addData("TIME REMAINING", (this.time - time_offset));
            telemetry.update();
        }
    }
    
    public void MoveTo(){
        
    }
    
    public void moveArm(){
    // setTargetPosition(int position)
    // pow = (-1 to 1)
        armMotor.setPower(1);
        armMotor.setTargetPosition((int) clamp(armMotor.getCurrentPosition() + Math.pow(gamepad2.left_stick_y*-1, 3) * 1000, 0.0, MAX_ENCODER_POSITION));
        telemetry.addData("IS IT MOVING TO TARGET POSITION:", armMotor.isBusy());
        telemetry.addData("TARGET POSITION:", armMotor.getTargetPosition());
    }
    double y;
    public void moveWrist(double currentPosition){
        
        //wristServo.setPosition(clamp(wristServo.getPosition() - gamepad2.right_stick_y * 0.0025,0,1));
        //wristServo.setPosition(clamp(wristServo.getPosition() - gamepad2.right_stick_y * 0.0025,.26,1));
        // .31 and .99
    //    y = clamp(0.0005*currentPosition + 1.0935,.26,1.05);
        
    //    wristServo.setPosition(y);
        telemetry.addData("servo: ", "i");//wristServo.getPosition()); 
        
    }
        
    public void moveFingers(){
        if (gamepad2.right_bumper){
            //fingersServo.setPosition(0);
        }
        else 
        {
          //fingersServo.setPosition(1);    
        }
    }
    double div_by = 1.6;
    public double relative_power(double intended_power)
    {
        return (13 * intended_power) / getVoltage();
    }
    
    public void MoveRobot(double[] direction, double rotation){
        
        // relative power sends the amount of power that would be sent when battery is at 13V
        double wheel0 = relative_power(clamp(-direction[0] - direction[1] - rotation, -1, 1));
        double wheel1 = relative_power(clamp(direction[0] - direction[1] + rotation, -1, 1));
        double wheel2 = relative_power(clamp(-direction[0] + direction[1] + rotation, -1, 1));
        double wheel3 = relative_power(clamp(direction[0] + direction[1] - rotation, -1, 1));
        
        MoveMotor(omniMotor0,wheel0);
        MoveMotor(omniMotor1,wheel1);
        MoveMotor(omniMotor2,wheel2);
        MoveMotor(omniMotor3,wheel3);
        
        
    }
    
    public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorPowerLogs.put(motor.getDeviceName(), new ArrayList<Double>());
         return motor;
     }
     
     public void MoveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
         motor.setPower(power);
         this.motorPowerLogs.get(motor.getDeviceName()).add(power);
         
     }
     public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
     }
     public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
        }
}
    // todo: write your code here

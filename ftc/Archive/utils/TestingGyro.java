

package org.firstinspires.ftc.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.*;

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
//@TeleOp(name="TestingGyro",group="Linear Opmode")

public class TestingGyro extends LinearOpMode{
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
    
    BNO055IMU imu;
    
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
    
    double targetRotation = 0;
    
    
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
        
        // Initialize each dictionary
        // arr = {0,7,8}
        // arr[1] = 2;
        // print (arr) -- {028}
        
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
        
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);        
        
        Gyro gyro = new Gyro(imu);
        
        //telemetry.addLine("asd");
        telemetry.update();
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart();
        
        targetPosition = ((((double) armMotor.getCurrentPosition()))) * -1 + armMotorOffset;
        armSpin.setDirection(DcMotor.Direction.FORWARD);
        while (this.opModeIsActive()) {
            // hit "X" button to break from loop and stop program
            // negative down, positive up 
            
            // Ternary operations
            
            double dpadX = 0;
            double dpadY = 0;
            
            
            // Makes a second mode for just moving the robot slowly
            if(gamepad1.dpad_right){dpadX += 0.4;}
            if(gamepad1.dpad_left){dpadX -= 0.4;}
            if(gamepad1.dpad_up){dpadY -= 0.4;}
            if(gamepad1.dpad_down){dpadY += 0.4;}
            
            if(gamepad2.a){targetRotation = 0;}
            if(gamepad2.b){targetRotation = Math.PI / 2;}
            if(gamepad2.x){targetRotation = Math.PI;}
            if(gamepad2.y){targetRotation = Math.PI / 180 * 270;}
            
            telemetry.addData("dpadX",dpadX);
            telemetry.addData("dpadY",dpadY);
            
            direction[0] = Math.pow(gamepad1.left_stick_x, 3) + dpadX;
            direction[1] = Math.pow(gamepad1.left_stick_y, 3) + dpadY;
            rotation = gamepad1.right_stick_x + Math.cbrt(gyro.deltaTheta(targetRotation) / (2 * (Math.PI)));
            
            telemetry.addData("theta",gyro.getTheta());
            telemetry.addData("deltaTheta", gyro.deltaTheta(targetRotation));
            
            MoveMotor(wheelMotor, gamepad2.right_trigger - gamepad2.left_trigger);
            MoveRobot(direction, rotation);
            currentPosition = ((((double) armMotor.getCurrentPosition()))) * -1 + armMotorOffset;
            moveArm();
            //moveWrist(currentPosition);
            //moveFingers()
            if (gamepad2.a && !gamepad2.b){
                armSpin.setPower(1);
            }
            else if (gamepad2.b && !gamepad2.a){
                armSpin.setPower(-1);
            }
            else{
                armSpin.setPower(0);
            }
            
            
            /*telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger);
            telemetry.addData("crServo", testServo.getPower());*/
            telemetry.addData("time", this.time);
            telemetry.addData("motor position: ",armMotor.getCurrentPosition());
            telemetry.addData("CR Servo Power: ",armSpin.getPower());
            //telemetry.addData("armPowerWithChange", armPowerWithChange);
            telemetry.update();
            //MoveRobot(direction, rotation);
            
            // Move servos 
            }
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
    public void MoveRobot(double[] direction, double rotation){
        
        double wheel0 = clamp(-direction[0] + direction[1] - rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] + rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] + rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] - rotation, -1, 1);
                
        MoveMotor(omniMotor0,wheel0/div_by);
        MoveMotor(omniMotor1,wheel1/div_by);
        MoveMotor(omniMotor2,wheel2/div_by);
        MoveMotor(omniMotor3,wheel3/div_by);
    }
        
    
        
    public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
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
}
    // todo: write your code here

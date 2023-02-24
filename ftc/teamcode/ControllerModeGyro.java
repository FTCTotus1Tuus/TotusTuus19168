package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
// @TeleOp(name="ControllerModeGyro",group="NonCompete")

public class ControllerModeGyro extends LinearOpMode{
    //Initialize Variables
    Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
    Boolean replaying = false;
    
    // Our transform (Set the values to default values)
    //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d,0d,0d});
    
    // For Movement
    // int, float, double 
    double[] direction = {0.0,0.0};
    double[] direction2 = {0.0,0.0};
    double rotation;
    
    
    // Motor 
    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right
    
    //LinearExtender
    DcMotor linearExtender;
    boolean wristCurrentPos = true;
    //Servos
    CRServo wristServo;
    CRServo grabServo;
    
    double targetPosition; 
    double MAX_ENCODER_POSITION = 1562;
        
    double EPSILON = 0.000000001;
    double armMotorOffset = 10;
    double currentPosition = 0;
    
    double armPower;
    
    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;
    private double          targetHeading = 0;

    
    public void runOpMode() {
        // connecting the motors to the config and setting to a variable name
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        
        
        
        
        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);
        linearExtender = initializeMotor("linearExtender");
        wristServo = hardwareMap.get(CRServo.class, "wristServo");
        grabServo = hardwareMap.get(CRServo.class, "grabServo");
        grabServo.setDirection(CRServo.Direction.REVERSE);
        
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        Files.test("org.firstinspires.ftc/teamcode/AutonomousData.txt", this);
        
        robotHeading = getRawHeading();
        
        
        telemetry.update();
        // Wait for the driver to start - must press play -- will run until driver presses 
        waitForStart();
        
        while (this.opModeIsActive()) {
            // hit "X" button to break from loop and stop program
            
            direction[0] = gamepad1.left_stick_x;
            direction[1] = gamepad1.left_stick_y;
            rotation = -1 * gamepad1.right_stick_x;
            
            
        
            MoveRobotGyro(direction, -rotation);
            //MoveRobot(direction, -rotation);
            
            
            telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
            telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger);
            
            telemetry.addData("time", this.time);
            
            
            //robotHeading = getRawHeading();
            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
            
            //telemetry.addData("armPowerWithChange", armPowerWithChange);
            telemetry.update();
          
            //stops claw
            if (gamepad1.x){
                //targetHeading = getRawHeading();
                targetHeading = getRawHeading();
                telemetry.addData("RESET BASE ANGLE (Gamepad_1:X)",0);
            }
            
            if (!(gamepad2.x | gamepad2.y)){
                grabServo.setPower(0);
            }
            //opens claw
            else if (gamepad2.y){
                grabServo.setPower(-0.35);
            }
            //closes claw
            else if(gamepad2.x){
                grabServo.setPower(0.35);
            }
            if (gamepad2.a){
                wristServo.setPower(1);
            }
            else if (gamepad2.b){
                wristServo.setPower(-1);
            }
            else {
                wristServo.setPower(0);
            }
            
            
            linearExtender.setPower(-gamepad2.left_stick_y/1.2);
            
            
            }
        }
    
    
  
    
    
        
   
    double div_by = 2;
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
        
    public void MoveRobotGyro(double[] direction, double rotation){
        
        robotHeading = (getRawHeading() - targetHeading) * Math.PI / 180;
        
        direction2[0] = Math.cos(robotHeading) * direction[0] - Math.sin(robotHeading) * direction[1];
        
        direction2[1] = Math.sin(robotHeading) * direction[0] + Math.cos(robotHeading) * direction[1];
        
        //rotation = (180 * rotation);
        
        //rotation = rotation - robotHeading;
        //if (rotation <= -180){rotation += 180;}
        //if (rotation >= 180){rotation -= 180;}

        //rotation = rotation/180.0;
        
        //telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
        //telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
        //telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
        //telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("d2X", direction2[0]);
        telemetry.addData("d2Y", direction2[1]);
        telemetry.addData("ARH", robotHeading);
        //telemetry.addData("CARH", targetHeading);
        
        
        
        
        double wheel0 = clamp(-direction2[0] + direction2[1] - rotation, -1, 1);
        double wheel1 = clamp(direction2[0] + direction2[1] + rotation, -1, 1);
        double wheel2 = clamp(-direction2[0] + -direction2[1] + rotation, -1, 1);
        double wheel3 = clamp(direction2[0] + -direction2[1] - rotation, -1, 1);
                
        MoveMotor(omniMotor0,wheel0/div_by);
        MoveMotor(omniMotor1,wheel1/div_by);
        MoveMotor(omniMotor2,wheel2/div_by);
        MoveMotor(omniMotor3,wheel3/div_by);
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
    

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


    

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }    
   
    
}





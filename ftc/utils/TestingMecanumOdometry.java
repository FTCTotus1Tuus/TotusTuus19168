package org.firstinspires.ftc.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import org.firstinspires.ftc.utils.Gyro;
import org.firstinspires.ftc.utils.MecanumOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.*;

import java.util.*;
import java.io.*;
import java.util.*;
import java.io.*;
// @TeleOp(name="TestingMecanumOdometry",group="Linear Opmode")
public class TestingMecanumOdometry extends LinearOpMode{
    //Initialize Variables
    Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
    Boolean playing = false;
        // Our transform (Set the values to default values)
    //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d,0d,0d});
    
    // For Movement
    double[] direction = {0.0,0.0};
    double rotation;
    double[] stop = {0.0,0.0};
    
    BNO055IMU imu;
    
    Gyro g;
    // Motor 
// Motor 
    DigitalChannel scooperLED; 
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
    
    MecanumOdometry odometry;
    
    // init LEDs 
    DigitalChannel scooperLED0Red;
    DigitalChannel scooperLED1Red;
    DigitalChannel scooperLED2Red;
    DigitalChannel scooperLED3Red;
    
    DigitalChannel scooperLED0Green;
    DigitalChannel scooperLED1Green;
    DigitalChannel scooperLED2Green;
    DigitalChannel scooperLED3Green;
    
    
    double targetPosition; // 
    double MAX_ENCODER_POSITION = 8700;
    static double CM_TO_PULSES = 14.288;
    static double PULSES_TO_CM = 0.0555;
    
    double EPSILON = 0.000000001;
    double armMotorOffset = 10;
    double currentPosition = 0;
    int[] ALPHAS = {0,0,0};
    double armPower;
    
    double PI = Math.PI;
    
    double time_offset = 0;
    // color/distance sensors
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ColorSensor extraSensorColor;
    DistanceSensor extraSensorDistance;
    
    
    
    double delay = 1.0;
    
    int encoder_offset = 0;
    
    public void runOpMode() {
        
        scooperLED0Red = hardwareMap.get(DigitalChannel.class, "scooperLED0Red");
        scooperLED1Red = hardwareMap.get(DigitalChannel.class, "scooperLED1Red");
        scooperLED2Red = hardwareMap.get(DigitalChannel.class, "scooperLED2Red");
        scooperLED3Red = hardwareMap.get(DigitalChannel.class, "scooperLED3Red");
        
        scooperLED0Green = hardwareMap.get(DigitalChannel.class, "scooperLED0Green");
        scooperLED1Green = hardwareMap.get(DigitalChannel.class, "scooperLED1Green");
        scooperLED2Green = hardwareMap.get(DigitalChannel.class, "scooperLED2Green");
        scooperLED3Green = hardwareMap.get(DigitalChannel.class, "scooperLED3Green");
        
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange2"); // use 2 because that is the only sensor used for blue 1
        extraSensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange2"); 
        extraSensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        
        
        /// VALUES FOR THE PID, CHANGE THEM HERE, IDK WHY BUT IF YOU WANT TO, YOU COULD CHANGE THEM FOR EACH MOTOR CONTROLLER
        
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelRange           = BNO055IMU.AccelRange.G2;
        parameters.magRate                 = BNO055IMU.MagRate.HZ10;
        parameters.magOpMode           = BNO055IMU.MagOpMode.REGULAR;
        parameters.magPowerMode        = BNO055IMU.MagPowerMode.NORMAL;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1);
        
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        
        
        armMotor = initializeMotor("armMotor"); 
        wheelMotor = initializeMotor("wheelMotor");
        //wristServo = hardwareMap.get(Servo.class, "wristServo");
        //fingersServo = hardwareMap.get(Servo.class, "fingersServo");
        armSpin = hardwareMap.get(CRServo.class, "armSpin");
        armMotor.setPower(1);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        omniMotor0.setDirection(DcMotor.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.REVERSE);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);
        //omniMotor0.setTargetPosition(-1000);
        //omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //omniMotor1.setTargetPosition(-1000);
        //omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //omniMotor2.setTargetPosition(1000);
        //omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //omniMotor3.setTargetPosition(1000);
        //omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        DcMotor omniMotors[] = new DcMotor[] {omniMotor0, omniMotor1, omniMotor2, omniMotor3};
        
        odometry = new MecanumOdometry(this);
        odometry.setMotors(omniMotors);
        odometry.reset();
        // Initialize each dictionary
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        while(this.opModeIsActive()){
        odometry.moveToPosition(50,21, 0);
        odometry.moveToPosition(50,31, 0);        //odometry.moveToPosition(100,100);
        odometry.moveToPosition(50,51, 0);        //odometry.moveToPosition(100,100);
        odometry.moveToPosition(50,21, 0);        //odometry.moveToPosition(100,100);
        odometry.moveToPosition(85,-5, 0);        //odometry.moveToPosition(100,100);
        odometry.moveToPosition(0,0, 0);        //odometry.moveToPosition(100,100);
        }
        
    }
    
    
    void SpinWheel(double start_time, double spin_time){
        double rotation;
        double[] direction = {0.0,0.0};
        while (timeBetween(start_time,start_time + spin_time))
            {
                
                direction[0] = 0;
                direction[1] = -.08;
                rotation = 0.08; 
                MoveRobot(direction, rotation);
                MoveMotor(wheelMotor,.55);
                
                telemetry.addData("time: ",this.time);
                telemetry.update();
            }
    }
    
    void stopRobot(){

        MoveRobot(stop, 0);
    }
    
    boolean timeBetween(double startTime, double endTime){
        if(this.time > startTime && this.time <= endTime){
            return true;
        }
        return false;
    }
    
    public double relative_power(double intended_power)
    {
        return (13 * intended_power) / getVoltage();
    }
    
    public int[] positionToEncoder(double delta_x, double delta_y){
        return new int[] {(int) Math.round((delta_x + delta_y) * CM_TO_PULSES), (int) Math.round((-delta_x + delta_y) * CM_TO_PULSES ), (int) Math.round((-delta_x + delta_y) * CM_TO_PULSES), (int) Math.round((delta_x + delta_y) * CM_TO_PULSES)};
    }
    
    public void MoveRobot(double[] direction, double rotation){
        
        
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
        
        // THIS LINE IS TO MAKE THE ROBOT RESIST FORCE WHEN 0 POWER IS APPLIED TO IT, I DONT LIKE THIS 
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
     }
     
     public void MoveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
         motor.setPower(power);
         
     }
     public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
     }
     public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
        }
    
   
}

// package org.firstinspires.ftc.NonUsefullFiles;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import java.util.Random;
// import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.*;


// import org.firstinspires.ftc.robotcore.external.Func;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Position;
// import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// import java.util.Locale;

// import org.firstinspires.ftc.utils.Gyro;

// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import java.lang.annotation.Target;
// import com.qualcomm.robotcore.*;

// import java.util.*;
// import java.io.*;

// @TeleOp(name="Blue2wPOSwTIMEwAlpha",group="Linear Opmode")

// public class Blue2wPOSwTIMEwAlpha extends LinearOpMode{
//     //Initialize Variables
//     Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
//     Boolean replaying = false;
    
    
//     // Our transform (Set the values to default values)
//     //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d,0d,0d});
    
//     // For Movement
//     double[] direction = {0.0,0.0};
//     double rotation;
//     double[] stop = {0.0,0.0};
    
//     BNO055IMU imu;
    
//     Gyro g;
//     // Motor 
// // Motor 
//     DigitalChannel scooperLED; 
//     DcMotor omniMotor0; // front left
//     DcMotor omniMotor1; // front right
//     DcMotor omniMotor2; // back left
//     DcMotor omniMotor3; // back right
    
//     DcMotor armMotor; // motor arm
//     DcMotor wheelMotor; // motor for da wheel
    
//     DcMotor hdHexMotor0; // arm control 
    
//     //Servo wristServo; // wrist movement - change to wrist servo
//     //Servo fingersServo; // fingers for grabbing - change to finger servo
//     CRServo armSpin; // CR Servo
    
    
//     // init LEDs 
//     DigitalChannel scooperLED0Red;
//     DigitalChannel scooperLED1Red;
//     DigitalChannel scooperLED2Red;
//     DigitalChannel scooperLED3Red;
    
//     DigitalChannel scooperLED0Green;
//     DigitalChannel scooperLED1Green;
//     DigitalChannel scooperLED2Green;
//     DigitalChannel scooperLED3Green;
    
    
    
    
    
//     double targetPosition; // 
//     double MAX_ENCODER_POSITION = 8700;
        
//     double EPSILON = 0.000000001;
//     double armMotorOffset = 10;
//     double currentPosition = 0;
//     int[] ALPHAS = {0,0,0};
//     double armPower;
    
//     double time_offset = 0;
//     // color/distance sensors
//     ColorSensor sensorColor;
//     DistanceSensor sensorDistance;
//     ColorSensor extraSensorColor;
//     DistanceSensor extraSensorDistance;
    
//     double delay = 1.0;
    
    
//     public void runOpMode() {
        
//         scooperLED0Red = hardwareMap.get(DigitalChannel.class, "scooperLED0Red");
//         scooperLED1Red = hardwareMap.get(DigitalChannel.class, "scooperLED1Red");
//         scooperLED2Red = hardwareMap.get(DigitalChannel.class, "scooperLED2Red");
//         scooperLED3Red = hardwareMap.get(DigitalChannel.class, "scooperLED3Red");
        
//         scooperLED0Green = hardwareMap.get(DigitalChannel.class, "scooperLED0Green");
//         scooperLED1Green = hardwareMap.get(DigitalChannel.class, "scooperLED1Green");
//         scooperLED2Green = hardwareMap.get(DigitalChannel.class, "scooperLED2Green");
//         scooperLED3Green = hardwareMap.get(DigitalChannel.class, "scooperLED3Green");
        
        
//         // get a reference to the color sensor.
//         sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange"); // use 2 because that is the only sensor used for blue 1
//         extraSensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange2");
//         // get a reference to the distance sensor that shares the same name.
//         sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange"); 
//         extraSensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange2"); 
//         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//         parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//         parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//         parameters.accelRange           = BNO055IMU.AccelRange.G2;
//         parameters.magRate                 = BNO055IMU.MagRate.HZ10;
//         parameters.magOpMode           = BNO055IMU.MagOpMode.REGULAR;
//         parameters.magPowerMode        = BNO055IMU.MagPowerMode.NORMAL;
//         parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//         parameters.loggingEnabled      = true;
//         parameters.loggingTag          = "IMU";
//         //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//         // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//         // and named "imu".
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//         imu.initialize(parameters);
        
//         imu.startAccelerationIntegration(new Position(), new Velocity(), 1);
        
//         omniMotor0 = initializeMotor("omniMotor0");
//         omniMotor1 = initializeMotor("omniMotor1");
//         omniMotor2 = initializeMotor("omniMotor2");
//         omniMotor3 = initializeMotor("omniMotor3");
//         armMotor = initializeMotor("armMotor");
//         wheelMotor = initializeMotor("wheelMotor");
//         //wristServo = hardwareMap.get(Servo.class, "wristServo");
//         //fingersServo = hardwareMap.get(Servo.class, "fingersServo");
//         armSpin = hardwareMap.get(CRServo.class, "armSpin");
//         armMotor.setPower(1);
//         armMotor.setTargetPosition(0);
//         armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
//         // armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
//         omniMotor0.setDirection(DcMotor.Direction.FORWARD);
//         omniMotor1.setDirection(DcMotor.Direction.REVERSE);
//         omniMotor2.setDirection(DcMotor.Direction.REVERSE);
//         omniMotor3.setDirection(DcMotor.Direction.FORWARD); 
        

        
        
        
//         //telemetry.addLine("asd");
//         telemetry.update();
//         armSpin.setDirection(DcMotor.Direction.FORWARD);
//         // Wait for the driver to start - must press play -- will run until driver presses 
        
        
//         initScooperLEDs();
//         waitForStart(); 
        
//         telemetry.addData("this.time", this.time);
//         telemetry.update();
//         g = new Gyro(imu);
// //***************************************************************************************** START AUTONOMOUS CODE ************************************************************


//         time_offset = this.time; 
//         // AutoMove(x-direction, y-direction, rotate, start_time, duration)
//         int spot_num = 0; 
//         // MoveY(intended_distance, power, start time)
        
//         double dist, new_time; 
//         double delay = 1;
        
//         scooperLEDsOnOrOff(false);
//         // move to the carousel
//         new_time = MoveXwArm(40,0.5,5000);
        
        
//         // SPOT #3
//         new_time = MoveY(17,0.25);
         
//         // check if te is there
//         // get_spot_num(spot we are checking, the current spot_num value)
//         stopRobot(delay); // pause for 0.5 seconds
         
//         spot_num = get_spot_num(3,spot_num); // evaluate distance
        
//         // SPOT #2
//         new_time = MoveY(25,0.25);
         
//         // check if te is there
//         // get_spot_num(spot we are checking, the current spot_num value)
//         stopRobot(delay); // pause for 0.5 seconds
        
//         spot_num = get_spot_num(2,spot_num); // evaluate distance
        
//         // SPOT #1
//         new_time = MoveY(24,0.25);
         
//         // check if te is there
//         // get_spot_num(spot we are checking, the current spot_num value)
//         stopRobot(delay); // pause for 0.5 seconds
         
//         spot_num = get_spot_num(1,spot_num); // evaluate distance 
        
        
//         // move closer to the tower 
//         new_time = MoveY(20, -0.5);
    
        
//         // turn so facing tower (rotate left) 
//         double turn_time = 0.86; 
//         AutoMove(0,0,-0.3, turn_time);
         
    
        
//         if (spot_num == 3){
//             // print spot num to phone
//             telemetry.addData("spot nom", spot_num);
//             telemetry.update();
//             // move fwd
//             new_time = MoveY(4.5,0.25);
//             // move arm to proper target position
//             double set_position = 2;
//             move_arm(7250,set_position);
            
        
//             // drop the block in the tower 
//             double spin_block = 1.2;
//             dropBlock(spin_block);
            
//         }
        
//         else if (spot_num == 2){
            
//             // print spot num to phone
//             telemetry.addData("spot nom", spot_num);
//             telemetry.update();
        
//             // move arm to proper target position
//             double set_position = 2;
//             move_arm(6725,set_position);
            
//             // move fwd
//             new_time = MoveY(5,-0.25);
        
//             // drop the block in the tower 
//             double spin_block = 1.2;
//             dropBlock(spin_block);
            
//         }
        
//         else {
//             telemetry.addData("spot nom", spot_num);
//             telemetry.update();
//                 // print spot num to phone
//             // move arm to proper target position
//             double set_position = 2;
//             move_arm(5950,set_position);
//             // move fwd
//             new_time = MoveY(15,-0.25);
//             // drop the block in the tower 
//             double spin_block = 1.2;
//             dropBlock(spin_block);
            
//         }
        
//         // go park and set arm home 
        
//         // rotate to the right 
//         double rotate = 1;
//         AutoMove(0,0,0.25,rotate);
        
        
//         // shift left 
//         new_time = MoveXwArm(65,-0.5,0);
        
//         // move fwd
//         new_time = MoveY(90,0.5);
        
//         // shift right 
//         new_time = MoveX(70,0.5); 
         
    
        
    
//         //turn_off_side_lights(); // turns of the sensor lights on the side of the robot 
        
        
        
    






// //***************************************************************************************** END AUTONOMOUS CODE *************************************************************
//         }
    



//       public void scooperLEDsOnOrOff(boolean state){
//         if(state == false){
//             scooperLED0Green.setState(false);
//             scooperLED1Green.setState(false);
//             scooperLED2Green.setState(false);
//             scooperLED3Green.setState(false);
//             scooperLED0Red.setState(true);
//             scooperLED1Red.setState(true);
//             scooperLED2Red.setState(true);
//             scooperLED3Red.setState(true);
//         }
//         else{
//             scooperLED0Red.setState(false);
//             scooperLED1Red.setState(false);
//             scooperLED2Red.setState(false);
//             scooperLED3Red.setState(false);
//             scooperLED0Green.setState(true);
//             scooperLED1Green.setState(true);
//             scooperLED2Green.setState(true);
//             scooperLED3Green.setState(true);
//         }
//     }
//     public void initScooperLEDs(){
//         scooperLED0Red.setMode(DigitalChannel.Mode.OUTPUT);
//         scooperLED1Red.setMode(DigitalChannel.Mode.OUTPUT);
//         scooperLED2Red.setMode(DigitalChannel.Mode.OUTPUT);
//         scooperLED3Red.setMode(DigitalChannel.Mode.OUTPUT);
        
//         scooperLED0Green.setMode(DigitalChannel.Mode.OUTPUT);
//         scooperLED1Green.setMode(DigitalChannel.Mode.OUTPUT);
//         scooperLED2Green.setMode(DigitalChannel.Mode.OUTPUT);
//         scooperLED3Green.setMode(DigitalChannel.Mode.OUTPUT);
        
//         scooperLED0Red.setState(false);
//         scooperLED0Green.setState(false);
//         scooperLED1Red.setState(false);
//         scooperLED1Green.setState(false);
//         scooperLED2Red.setState(false);
//         scooperLED2Red.setState(false);
//         scooperLED3Green.setState(false);
//         scooperLED3Green.setState(false);
//     }
    
    
//     public int get_spot_num(int spot_test_num, int spot_num){
//         // function will record the alpha at each spot, 
//         // ALPHAS holds the alpha at each spot 
//         int alpha = sensorColor.alpha();
//         ALPHAS[spot_test_num - 1] = alpha; // set current alpha to corresponding index pos 
//         int alphas_greater_than_count = 0; // count how many times the current alpha is greater than the previous alpha
//         // since the ALPHAS array is initially all 0s, we can see how many times the alpha is greater than the previous alphas
//         // also, it will work with all spot nums because of this
//         /* e.g. if the TE is in spot 1, that will be the greatest alpha value in the array, thus alphas_greater_than_count
//         will never equal 3 */ 
//         // e.g. if the TE is in spot 2, the initial spot num will be #1, but the spot 2 alpha val will be greater than 
//         // spot 1, equal to itself, and greater than spot 3, thus it will be the final spot num 
//         for (int i = 0; i < 3; i++){
//             // for each alpha, check if the alpha is greater than all other alphas or equal to itself 
//             if (alpha >= ALPHAS[i])
//                 alphas_greater_than_count += 1; 
//         }
        
//         // print data to phone 
//         if (alphas_greater_than_count == 3){
//             spot_num = spot_test_num; 
//             scooperLEDsOnOrOff(true);
//         }
//         else {
//             scooperLEDsOnOrOff(false);
//         }
//         // else, just stay what the spot num was before 
        
        
//         telemetry.addData("alphas_greater_count", alphas_greater_than_count);
//         telemetry.addData("alpha", alpha);
//         telemetry.addData("spot_test_num", spot_test_num);
//         telemetry.addData("spot_num", spot_num);
//         telemetry.update();
        
        
    
//         return spot_num; 
        
        
//     } 
    
//     public void dropBlock(double spin_time){
//         double start_time = this.time; 
//         while (timeBetween(start_time, start_time + spin_time)){
//             armSpin.setPower(-0.8);
//         }
//         armSpin.setPower(0);
//     }
    
//     public double MoveYwBlink(double intended_position, double power, int spot_num){
//         double start_time = this.time;
//         double t = 0;
//         if ((power == 0.25) || (power == -0.25)){
//             t = 0.029*intended_position + 0.0075;
//             AutoMoveWithBlink(0,power,0,t,spot_num); 
//         }
//         else if ((power == 0.5)||(power == -0.5)){
//             t = 0.0129*intended_position + 0.0252;
//             AutoMoveWithBlink(0,power,0,t,spot_num);
//         }
//         stopRobot(0);
//         return t;
//     }
            
//     public double MoveY(double intended_position, double power){
//         double start_time = this.time;
//         double t = 0;
//         if ((power == 0.25) || (power == -0.25)){
//             t = 0.029*intended_position + 0.0075;
//             AutoMove(0,power,0,t); 
//         }
//         else if ((power == 0.5)||(power == -0.5)){
//             t = 0.0129*intended_position + 0.0252;
//             AutoMove(0,power,0,t);
//         }
//         stopRobot(0);
//         return t;
//     }

//     public double MoveX(double intended_position, double power){
//         double start_time = this.time;
//         double t = 0;
//         if ((power == 0.25)||(power==-0.25)){
//             t = 0.0456*intended_position + 0.0089;
//             AutoMove(power,0,0,t);
//         }
//         else if ((power == 0.5)||(power==-0.5)){
//             t = 0.0186*intended_position + 0.1851;
//             AutoMove(power,0,0,t);
//         }
//         stopRobot(0);
//         return t;
//     }
    
//     public double MoveXwArm(double intended_position, double power, int target_pos){
//         double start_time = this.time; 
//         double t = 0;
//         if ((power == 0.25)||(power==-0.25)){
//             t = 0.0456*intended_position + 0.0089;
//             AutoMoveWithArm(power,0,0,t,target_pos);
//         }
//         else if ((power == 0.5)||(power==-0.5)){
//             t = 0.0186*intended_position + 0.1851;
//             AutoMoveWithArm(power,0,0,t,target_pos);
//         }
//         stopRobot(0);
//         return t;
//     }
    
    
//     public double MoveYwArm(double intended_position, double power, int target_pos){
//         double start_time = this.time; 
//         double t = 0;
//         if ((power == 0.25) || (power == -0.25)){
//             t = 0.029*intended_position + 0.0075;
//             AutoMoveWithArm(0,power,0,t,target_pos); 
//         }
//         else if ((power == 0.5)||(power == -0.5)){
//             t = 0.0129*intended_position + 0.0252;
//             AutoMoveWithArm(0,power,0,t,target_pos);
//         }
//         stopRobot(0);
//         return t;
        
//     }
    
    
//     void move_arm(int target_pos,  double set_position){
//         double start_time = this.time; 
//         while(timeBetween(start_time,start_time+set_position)){
//                 armMotor.setTargetPosition(target_pos);
//             }
//     }
        
    
//     double get_dist(){ 
//         double dist = sensorDistance.getDistance(DistanceUnit.CM);
//         telemetry.addData("DISTANCE",dist);
//         telemetry.update();
//         return dist; 
//     }
    
//     void AutoMoveWithBlink(double x, double y, double r , double duration, int spot_num){
//         double start_time = this.time; 
        
//         double[] direction = {0.0,0.0};
//         String targetPosition = "null";
//         String armPower = "null";
        
//         direction[0] = x;
//         direction[1] = y;
//         rotation = r; 
        
//         telemetry.addData("this.time", this.time);
        
//         //telemetry.addData("heading", )
//         scooperLEDsOnOrOff(true);
//         telemetry.update();
//         double end_time = start_time + duration;
//         while (timeBetween(start_time, end_time)){
//             MoveRobot(direction, rotation);
            
//             /* if spot number 1 
//             if ((spot_num == 1) && (timeBetween(start_time, (end_time - (duration/2)))) ){
//                 scooperLEDsOnOrOff(true);
//             }
//             else{
//                 scooperLEDsOnOrOff(false);
//             }*/
            
            
            
//             }
        
//         stopRobot(0);
//     }
    
//     void AutoMoveWithArm(double x, double y, double r , double duration, int target_pos)
//     {
//         double start_time = this.time; 
        
//         double[] direction = {0.0,0.0};
//         String targetPosition = "null";
//         String armPower = "null";
        
//         direction[0] = x;
//         direction[1] = y;
//         rotation = r; 
        
//         telemetry.addData("this.time", this.time);
        
//         //telemetry.addData("heading", )
        
//         telemetry.update();
        
//         while (timeBetween(start_time, start_time + duration)){
//             MoveRobot(direction, rotation);
//             armMotor.setTargetPosition(target_pos);}
        
//         stopRobot(0);
        
            
//     }
    

//     void AutoMove(double x, double y, double r , double duration)
//     {
//         double start_time = this.time ; 
        
//         double[] direction = {0.0,0.0};
//         String targetPosition = "null";
//         String armPower = "null";
        
//         direction[0] = x;
//         direction[1] = y;
//         rotation = r; 
        
//         telemetry.addData("this.time", this.time);
//         telemetry.update();
        
//         while (timeBetween(start_time, start_time + duration))
//             MoveRobot(direction, rotation);
        
//         stopRobot(0);
            
//     }
    
//     void AutoMoveWithGyro(double x, double y, double r, double duration){
//         double start_time = this.time; 
        
//         double[] direction = {0.0,0.0};
//         String targetPosition = "null";
//         String armPower = "null";
        
//         direction[0] = x;
//         direction[1] = y;
//         rotation = -Math.cbrt(g.deltaTheta(r))/5; 
        
//         telemetry.addData("this.time", this.time);
//         telemetry.addData("deltaTheta", g.deltaTheta(r));
//         telemetry.addData("theta", g.getTheta());
//         telemetry.update();
        
//         while (timeBetween(start_time, start_time + duration))
            
//         telemetry.addData("this.time", this.time);
//         telemetry.addData("deltaTheta", g.deltaTheta(r));
//         telemetry.addData("theta", g.getTheta());
//         telemetry.update();
//             rotation = -Math.cbrt(g.deltaTheta(r))/5; 
//             MoveRobot(direction, rotation);
        
//         stopRobot(0);
            
//     }
    
    
    
//     void SpinWheel( double spin_time){
//         double start_time = this.time; 
//         double rotation;
//         double[] direction = {0.0,0.0};
//         while (timeBetween(start_time,start_time + spin_time))
//             {
                
//                 direction[0] = 0;
//                 direction[1] = -.04;
//                 rotation = 0.04; 
//                 MoveRobot(direction, rotation);
//                 MoveMotor(wheelMotor,.65);
                
//                 telemetry.addData("time: ",this.time);
//                 telemetry.update();
//             }
//         MoveMotor(wheelMotor,0);
//     }
    
//     void stopRobot( double delay){
//         double start_time = this.time; 
//         if ((start_time == 0) || (delay == 0)){
//             MoveRobot(stop,0);
//         }
//         while (timeBetween(start_time,start_time + delay)){
//             MoveRobot(stop, 0);}
//     }
    
//     boolean timeBetween(double startTime, double endTime){
//         if((this.time  >= startTime) && (this.time <= endTime)){
//             return true;
//         }
//         return false;
//     }
    
    
//     public double relative_power(double intended_power)
//     {
//         return (13 * intended_power) / getVoltage();
//     }
    
//     public void MoveRobot(double[] direction, double rotation){
        
//         // relative power sends the amount of power that would be sent when battery is at 13V
//         double wheel0 = relative_power(clamp(-direction[0] - direction[1] - rotation, -1, 1));
//         double wheel1 = relative_power(clamp(direction[0] - direction[1] + rotation, -1, 1));
//         double wheel2 = relative_power(clamp(-direction[0] + direction[1] + rotation, -1, 1));
//         double wheel3 = relative_power(clamp(direction[0] + direction[1] - rotation, -1, 1));
        
//         MoveMotor(omniMotor0,wheel0);
//         MoveMotor(omniMotor1,wheel1);
//         MoveMotor(omniMotor2,wheel2);
//         MoveMotor(omniMotor3,wheel3);
//     }
    
//     public DcMotor initializeMotor(String name){
//          /*This is just a handy dandy function which saves a few lines and looks cool,
//          it initializes the motor and it also initializers the motor power logs for this motor*/
//         DcMotor motor = hardwareMap.get(DcMotor.class, name);
//         motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         this.motorPowerLogs.put(motor.getDeviceName(), new ArrayList<Double>());
//          return motor;
//      }
     
//      public void MoveMotor(DcMotor motor, double power){
//          /*This function just moves the motors and updates the
//          logs for replay*/
//          motor.setPower(power);
//          this.motorPowerLogs.get(motor.getDeviceName()).add(power);
         
//      }
//      public static double clamp(double val, double min, double max) {
//         return Math.max(min, Math.min(max, val));
//      }
//      public double getVoltage() {
//         return (hardwareMap.voltageSensor.iterator().next().getVoltage());
//         }
    
    
    
    
    
    
//     }
//     // todo: write your code here

// package org.firstinspires.ftc.Archive;

// //import org.firstinspires.ftc.teamcode.Launcher;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import java.util.Random;
// import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.*;

// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import java.lang.annotation.Target;
// import com.qualcomm.robotcore.*;
// //
// import java.util.*;
// import java.io.*;

// //@TeleOp(name="MainwithLauncher", group="Linear Opmode")
// public class TheEverything extends LinearOpMode {
//      // Initializing variables 
//      Hashtable<String, List<Double>> motorPowerLogs = new Hashtable<String, List<Double>>();
     
//      //FileWriter fileWriter = new FileWriter("./latestLog.txt");
//      Boolean replaying = false; 
     
//      // Our transfrom (Set the values to default values)
//      //Transform transform = new Transform(new double[] {0d,0d,0d}, new double[] {0d, 0d, 0d});
     
//      // Motors
//      DcMotor coreHexMotor0; // Left back wheel
//      DcMotor coreHexMotor1; // Right back wheel
//      DcMotor hdHexMotor2; // Arm
//      DcMotor hdHexMotor3; // Launcher
     
//      // Servo
     
//      CRServo servo0; // For the treads
//      CRServo servo1; // For the brush
//      Servo servo5; // For the arm
     
//      // Sensors
//      NormalizedColorSensor colorSensor;
//      TouchSensor touchSensor;
     
//      // Randomizer
//      Random random = new Random();

//      @Override
//      public void runOpMode() {
         
//          // Finding the motors from the config and setting them to the coreHexMotor variables
//          coreHexMotor0 = initializeMotor("coreHexMotor0");
//          coreHexMotor1 = initializeMotor("coreHexMotor1");
//          hdHexMotor2 = initializeMotor("hdHexMotor2");
//          hdHexMotor3 = initializeMotor("hdHexMotor3");
         
         
//          servo0 = hardwareMap.get(CRServo.class, "servo0");
//          servo1 = hardwareMap.get(CRServo.class, "servo1");
//          servo5 = hardwareMap.get(Servo.class, "servo5");
         
//          coreHexMotor0.setDirection(DcMotor.Direction.REVERSE);
//          coreHexMotor1.setDirection(DcMotor.Direction.FORWARD);
         
//          // Initialize the sensors
//          colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//          touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
         
//          // Initialize each dictionary
//          telemetry.addData("Status", "Initialized");
//          telemetry.update();
         
//          // Wait for the game to start (driver presses PLAY)
//          waitForStart();
         
//          // run until the end of the match (driver presses STOP)
         
//          while (this.opModeIsActive()) {
//              // If you hit the x button on the gamepad it will
//              // break from the while loop and stop the program.
//              if(this.gamepad1.x){
//                   break;
//               }
//              if(this.gamepad1.y){
//                  replaying = !replaying;
//              }
//              if(!replaying){
//                  moveRobot();
//                  scooper();
//                  arm();
//                  launcher();
//              }
//              else{
//                  if(motorPowerLogs.get(coreHexMotor0.getDeviceName()).size() > 1){
//                      MoveMotor(coreHexMotor0,motorPowerLogs.get(coreHexMotor0.getDeviceName()).get(0));
//                      MoveMotor(coreHexMotor1,motorPowerLogs.get(coreHexMotor1.getDeviceName()).get(0));
//                      MoveMotor(hdHexMotor2,motorPowerLogs.get(hdHexMotor2.getDeviceName()).get(0));
//                      motorPowerLogs.get(coreHexMotor0.getDeviceName()).remove(0);
//                      motorPowerLogs.get(coreHexMotor1.getDeviceName()).remove(0);
//                      motorPowerLogs.get(hdHexMotor2.getDeviceName()).remove(0);
//                  }
//                  else{
//                      replaying = !replaying; 
//                  }
//              }
             
//              // Make it so that LT & RT map to the big and smaller arm
             
//              telemetry.addData("gamepad1.right_bumper", gamepad1.right_bumper);
//              telemetry.addData("servo0.getPower()", servo0.getPower());
//              // Showing time online for bug-testing
//              telemetry.addData("Time Online", this.time);
                 
//              // Adding data about the core hex motor 0
//              telemetry.addLine("Core Hex Motor 0\n");
//              telemetry.addData("Target Power", motorPowerLogs.get(coreHexMotor0.getDeviceName()).get(motorPowerLogs.get(coreHexMotor0.getDeviceName()).size() - 1));
            
//              telemetry.addData("Motor Power", coreHexMotor0.getPower());    

//              // Adding data about the core hex motor 1
//              telemetry.addLine("Core Hex Motor 1\n");
//              telemetry.addData("Target Power", motorPowerLogs.get(coreHexMotor1.getDeviceName()).get(motorPowerLogs.get(coreHexMotor1.getDeviceName()).size() - 1));
             
//              telemetry.addData("Motor Power", coreHexMotor1.getPower());
             
//              telemetry.addData("Dpad_Up", gamepad1.dpad_up);
//              telemetry.addData("Dpad_Down", gamepad1.dpad_down);
             
//              // Adding data about our transform
//              telemetry.addData("Total Power for both motors", Double.toString(sum(motorPowerLogs.get(coreHexMotor0.getDeviceName()))) + ", " + Double.toString(sum(motorPowerLogs.get(coreHexMotor1.getDeviceName()))));
//              //telemetry.addData("Transform:\n", "Coords:\n\t" + this.transform.coords.toString() + "\nRotation:\n\t" + this.transform.rotation.toString());
             
//              // Updates all of the data we added
//              telemetry.update();
//          }
       
//      } 
     
//      public DcMotor initializeMotor(String name){
//          /*This is just a handy dandy function which saves a few lines and looks cool,
//          it initializes the motor and it also initializers the motor power logs for this motor*/
         
//         DcMotor motor = hardwareMap.get(DcMotor.class, name);
//         this.motorPowerLogs.put(motor.getDeviceName(), new ArrayList<Double>());
//          return motor;
//      }
     
//      public void MoveMotor(DcMotor motor, double power){
//          /*This function just moves the motors and updates the
//          logs for replay*/
         
//          motor.setPower(power);
//          this.motorPowerLogs.get(motor.getDeviceName()).add(power);
         
//      }
     
//      public void moveRobot(){
//          /* Get gamepad data, then we do some epic math to calculate what numbers
//          to send to the robot to get it to move how we want to */
         
//          MoveMotor(coreHexMotor0,Math.pow(gamepad1.left_stick_x - gamepad1.left_stick_y, 3));
//          MoveMotor(coreHexMotor1,Math.pow(-gamepad1.left_stick_x - gamepad1.left_stick_y, 3));
         
         
//          if(gamepad1.right_bumper){
//              if(gamepad1.left_bumper){
//                  MoveMotor(hdHexMotor3,1f);
//              }
//              else{
//                  MoveMotor(hdHexMotor3, 0.5f);
//              }
             
         
//          }
//          else{
//              MoveMotor(hdHexMotor3,0.0f);
//          }
         
     
//      }
     
//      public class Launcher{}{
         
//          MoveMotor(hdHexMotor3, gamepad1.right_trigger);
//      }
//      public void launcher(){
//          // ALl of the necessary code for the launcher
//         MoveMotor(hdHexMotor3, gamepad1.right_trigger);
         
//      }
     
//      public void scooper(){
//          // All of the necessary code for the scooper (treadmil and brush)
//          if(gamepad1.right_bumper){
//             servo0.setPower(-1);
//         }
//         else{
//             servo0.setPower(0);
//         }
//         if(gamepad1.left_bumper){
//             servo1.setPower(-1);
//         }
//         else{
//             servo1.setPower(0);
//          }
         
//      }
     
//      public void arm(){
//          // All of the necessary code for the arm
         
//          // Main arm
//          if(gamepad1.dpad_down){
//              MoveMotor(hdHexMotor2, 0.5f);
//          }
//          else if(gamepad1.dpad_up){
//              MoveMotor(hdHexMotor2, -0.5f);
//          }
//          else{
//              MoveMotor(hdHexMotor2, 0f);
//          }
         
//          // Arm at the end
//          if(gamepad1.right_stick_y == 0){
//             servo5.setPosition(servo5.getPosition());
//             }
//          else{
//              servo5.setPosition(servo5.getPosition() + gamepad1.right_stick_y * -0.005);
//          }
//     }
         
         
     
     
//      public double[] calculateDistanceBetweenRobotToTransform(Transform other){
//          return new double[] {this.transform.coords.x - other.coords.x, this.transform.coords.y - other.coords.y, this.transform.coords.z - other.coords.z};
//      }
     
//      public double sum(List<Double> arr){
//         /* Sums an array and returns value*/
        
//         double total = 0;
//         for(double num : arr){
//             total += num;
//         }
//         return total;
//     }
    
//     public static double clamp(double val, double min, double max) {
//         return Math.max(min, Math.min(max, val));
//     }
    
//      // Caluclating the Yaw, Pitch, Roll to the other object
//      public double calculateYawToObject(Transform other){return Math.atan((this.transform.coords.x - other.coords.x) / (this.transform.coords.z - other.coords.z)) * (180/Math.PI);}
     
//      public double calculatePitchToObject(Transform other){return Math.atan((this.transform.coords.y - other.coords.y) / (this.transform.coords.x - other.coords.x)) * (180/Math.PI);}
     
//      public double calculateRollToObject(Transform other){return Math.atan((this.transform.coords.y - other.coords.y) / (this.transform.coords.x - other.coords.x)) * (180/Math.PI);}
     
// }

// class GameFieldObjects{
//     // This class will hold all of the game field objects and their relative positions
//     Transform ball = new Transform(new double[] {0,10,5}, null);
    
// }

// /********** HELPER CLASSES **********/

// class Transform{
//     /* This class is for everything having to do with transform, including position,
//     scale, rotation, etc. We will use this to make objects and give them positions so that
//     we can use aimbot*/
    
//     // Initialize variables
    
//     Coords coords;
//     EulerRotation rotation;
    
//     // Initialize the class
//     Transform(double[] coords, double[] rotation){
//         // Set our coords and rotation using the Coords and Rotation class
//         // Check if its null becuase sometimes we don't want to pass any values
//         if (coords != null)
//             this.coords = new Coords(coords);
//         if (rotation != null)
//             this.rotation = new EulerRotation(rotation);
//     }
    
//     public void updateTransform(double[] coords, double[] rotation){
//         /* Updates the transform */
        
//         this.coords.updateCoords(coords);
//         this.rotation.updateRotation(rotation);
//     }
    
//     public void addTransform(Transform other){
//         /* Updates the transform but it does that while adding values to it so less work for us in the future */
        
//         this.coords.add(other.coords.coords());
//         this.rotation.add(other.rotation.rotation());
//     }
// }

// class Coords{
//     /* This class holds all of the coordinate variables and the like and we're going to be using feet*/
    
//     // Initialize variables
    
//     double x = 0;
//     double y = 0;
//     double z = 0;

//     // Initialize the class
//     Coords(double[] values){
//         this.x = values[0];
//         this.y = values[1];
//         this.z = values[2];
//     }
    
//     public double[] coords(){
//         /* Returns the coords */
        
//         return new double[] {this.x, this.y, this.z};
//     }
    
//     public void updateCoords(double[] values){
//         /* Update the coords */
        
//         this.x = values[0];
//         this.y = values[1];
//         this.z = values[2];
//     }
    
//     public void add(double[] values){
//         /* This function will take a 3 length array and add its values to our coords */
        
//         this.x += values[0];
//         this.y += values[1];
//         this.z += values[2];
//     }
    
//     public String toString(){
//         /* Return a string of the coords in format: "x, y, z" */
        
//         return Double.toString(this.x) + ", " + Double.toString(this.y) + ", " + Double.toString(this.z);
//     }
// }

// class EulerRotation{
//     /* This class will be holding all of the rotation variables and the like.
//     Everything is in degrees.*/
    
//     // Initialize variables
    
//     double roll; // Roll is turning in x direction
//     double pitch; // Pitch is turning in the y direction
//     double yaw; // Yaw is turning in the z direction
    
//     // Initialize the class
//     EulerRotation(double[] values){
//         this.roll = values[0];
//         this.pitch = values[1];
//         this.yaw = values[2];
//     }
    
//     public double[] rotation(){
//         /* Returns the rotation*/
        
//         return new double[] {this.roll, this.pitch, this.yaw};
//     }
    
//     public void updateRotation(double[] values){
//         /* Update the coords */
        
//         this.roll = values[0];
//         this.pitch = values[1];
//         this.yaw = values[2];
//     }
    
//     public void add(double[] values){
//         /* This function will take a 3 length array and add its values to the rotation*/
        
//         this.roll += values[0];
//         this.pitch += values[1];
//         this.yaw += values[2];
//     }
    
//     public String toString(){
//         /* Returns a string version of our rotation in format of "roll, pitch, yaw"*/
        
//         return Double.toString(this.roll) + ", " + Double.toString(this.pitch) + ", " + Double.toString(this.yaw);
//     }
// }

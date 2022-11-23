
// package org.firstinspires.ftc.NonUsefullFiles;

// import android.app.Activity;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import android.graphics.Color;
// import android.view.View;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;

// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// import java.util.Locale;


// @Autonomous(name="SensorTest",group="Linear Opmode")
// public class SensorTest extends LinearOpMode {


//     ColorSensor sensorColor;
//     DistanceSensor sensorDistance;

//     ColorSensor sensorColor2;
//     DistanceSensor sensorDistance2;

//     @Override
//     public void runOpMode() {

//         waitForStart();

//         while (opModeIsActive()) {
//             // convert the RGB values to HSV values.
//             // multiply by the SCALE_FACTOR.
//             // then cast it back to int (SCALE_FACTOR is a double)
//             Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                           (int) (sensorColor.green() * SCALE_FACTOR),
//                           (int) (sensorColor.blue() * SCALE_FACTOR),
//                             hsvValues);

//             Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
//                           (int) (sensorColor2.green() * SCALE_FACTOR),
//                           (int) (sensorColor2.blue() * SCALE_FACTOR),
//                             hsvValues2);

            

//             double Dis = sensorDistance.getDistance(DistanceUnit.CM);
//             double Dis2 = sensorDistance2.getDistance(DistanceUnit.CM);
            
//             double awayFromCenter = (Dis/2) - (Dis2/2);



//             // send the info back to driver station using telemetry function.

//             // telemetry.addData("Distance (cm)",
//             //         String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//             // telemetry.addData("Alpha", sensorColor.alpha());
//             // telemetry.addData("Red  ", sensorColor.red());
//             // telemetry.addData("Green", sensorColor.green());
//             // telemetry.addData("Blue ", sensorColor.blue());
//             // telemetry.addData("Hue", hsvValues[0]);
//             // telemetry.update();
//         }
//     }
// }

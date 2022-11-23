package org.firstinspires.ftc.utils;


import java.io.BufferedReader;  
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.io.File;
import java.util.Scanner;
import java.io.FileReader;  
import java.io.IOException;  

import java.util.*;
import java.io.*;

public class Files {
    
  public static void test(String filename, LinearOpMode lom){
      File directoryPath = new File("AmongUs.txt");
      String contents[] = {directoryPath.getAbsolutePath()};
      lom.telemetry.addLine("FILES:");
      for(int i = 0; i < contents.length; i++){
          lom.telemetry.addLine(contents[i]);    
      }
      
  }
    
  /*This class will be able to read and write file data, we will use it to load and save autonomous data*/    
  public static String read(String filename, LinearOpMode lom) {
      /*Reads txt data and returns it as a string*/
      String data = "";
        try {
          File myObj = new File(filename);
          Scanner myReader = new Scanner(myObj);
          while (myReader.hasNextLine()) {
            data = data + myReader.nextLine();
          }
          myReader.close();
        } catch (FileNotFoundException e) {
            lom.telemetry.addLine(e.getMessage());
            lom.telemetry.addLine(System.getProperty("user.dir")); //e.getMessage()
        }
    return data;
  }
  
  public static LinkedList<String[]> readCSV(String filename) {
      /*Reads the CSV, and returns it as a LinkedList of a string Array*/
    String line = "";  
    String splitBy = ",";  
    LinkedList<String[]> data = new LinkedList<String[]>();
    //parsing a CSV file into BufferedReader class constructor  
    try{
    BufferedReader br = new BufferedReader(new FileReader(filename));  
    while ((line = br.readLine()) != null)   //returns a Boolean value  {  
        data.add(line.split(splitBy));    // use comma as separator  
    }
    catch (Exception e) {
        
    }
    return data;
  }
 
}
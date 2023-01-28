package org.firstinspires.ftc.utils;


public class Utils {
    
    static double roundSigFigs(double num, int sigfigs){
        return (Math.round(num * Math.pow(10, sigfigs-1)) * Math.pow(10, -(sigfigs-1)));
    }
    
     public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}


class Coords{
    /* This class holds all of the coordinate variables and the like and we're going to be using feet*/
    
    // Initialize variables
    double x = 0;
    double y = 0;

    // Initialize the class
    Coords(double[] values){
        this.x = values[0];
        this.y = values[1];
    }
    
    public double[] coords(){
        /* Returns the coords */
        return new double[] {this.x, this.y};
    }
    
    public void updateCoords(double[] values){
        /* Update the coords */
        this.x = values[0];
        this.y = values[1];
    }
    
    public void add(double[] values){
        /* This function will take a 3 length array and add its values to our coords */
        this.x += values[0];
        this.y += values[1];
    }
    
    public String toString(){
        /* Return a string of the coords in format: "x, y, z" */
        return "Coordinates: ("+Double.toString(this.x) + ", " + Double.toString(this.y) + ")\n";
    }
}

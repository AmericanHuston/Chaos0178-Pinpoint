package org.firstinspires.ftc.teamcode.RobotBoard;

public class LocalUtils {
    public static double clamp(double min, double max, double input){
        if (input > max){
            input = max;
        } else if (input < min) {
            input = min;
        }
        return input;
    }
}

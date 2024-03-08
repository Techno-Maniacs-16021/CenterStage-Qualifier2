package org.firstinspires.ftc.teamcode.auto;

public class AutonConstants {
    public enum AutonType{
        BLUE_CLOSE_2_PLUS_0,
        BLUE_FAR_2_PLUS_1,
        RED_CLOSE_2_PLUS_0,
        RED_FAR_2_PLUS_1,
    }
    public static boolean isClose(AutonType autonType){
        return autonType == AutonType.RED_CLOSE_2_PLUS_0 || autonType == AutonType.BLUE_CLOSE_2_PLUS_0;
    }
    public static boolean isBlue(AutonType autonType){
        return  autonType == AutonType.BLUE_CLOSE_2_PLUS_0 || autonType == AutonType.BLUE_FAR_2_PLUS_1;
    }
}

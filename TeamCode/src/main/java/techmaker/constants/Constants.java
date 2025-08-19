package techmaker.constants;

public class Constants {
    public static class Drivetrain {
        public static String RightFront = "leftFront";
        public static String LeftFront = "rightFront";
        public static String RightBack = "leftRear";
        public static String LeftBack = "rightRear";
    }
    public static class Intake {
        public static String Right = "rightintake";
        public static String Left = "leftintake";
        public static String Middle = "middleintake";
        public static String LeftSlider = "leftslider";
        public static String RightSlider = "rightslider";
        public static String LeftWrist = "leftwrist";
        public static String RightWrist = "rightwrist";
        public static double RedMin = 10.0;
        public static double RedMax = 50.0;
        public static double YellowMin = 51.0;
        public static double YellowMax=110.0;
        public static double BlueMin = 180.0;
        public static double BlueMax = 250.0;
        public enum SampleColor {Red,Yellow,Blue,Idle};


    }
    public static class Claw {
        public static String RightArm = "clawrightarm";
        public static String LeftArm = "clawleftarm";
        public static String RightClaw = "clawright";
        public static String LeftClaw = "clawleft";
        public static String MiddleClaw = "clawmiddle";
    }
    public static class Elevator {
        public static String LeftElevator = "leftelevator";
        public static String RightElevator = "rightelevator";
    }
}

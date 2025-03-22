package frc.robot;

public class Constant {
    //Here's the constants of the Controller's IDs and buttons.
      public static class ButtonConst {
      public static final int driverControllerID = 0; 
      public static final int operatorControllerID = 1;
      public static final int A = 1; 
      public static final int B = 2;
      public static final int X = 3;
      public static final int Y = 4;
      public static final int L1 = 5;
      public static final int R1 = 6;
      public static final int Back = 7;
      public static final int Start = 8;
      public static final int L3 = 9;
      public static final int R3 = 10;
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int RX = 4;
      public static final int RY = 5;

      }
      //Here's the constants of the Motors's IDs.
      public static class MotorsId {
        public static final int m_leftChassisSparkID = 5;
        public static final int m_rightChassisSparkID = 2;
        public static final int f_leftChassisSparkID = 7;
        public static final int f_rightChassisSparkID = 3;
        public static final int m_climberSparkID = 4;
        public static final int f_climberSparkID = 6;
        public static final int m_intakerSparkID = 9;
        public static final int f_intakerSparkID = 8;
      }

      //Here's the constants of the Solenoid's IDs.
      public static class PneumaticsdId{
        public static final int wristOpenerFwdID = 14;
        public static final int wristOpenerRvsID = 15;
        public static final int wristRaiserFwdID = 12;
        public static final int wristRaiserRvsID = 13;
        public static final int transmissionFwdID = 0;
        public static final int transmissionRvsID = 1;

      }

      public static class Config{
        public static final int curLimit = 40;
        public static final double rampRate = 0.5;
        public static final double Tdelay = 0.5;
      }

   
}



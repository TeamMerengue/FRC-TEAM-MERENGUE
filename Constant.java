package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public final class Constants {
  
  //***Controls Map***
  public static class ControllerConstants {
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

  //***Motors ID Map***
  public static class MotorsId {
    public static final int m_leftChassisSparkID = 2;
    public static final int m_rightChassisSparkID =5;
    public static final int f_leftChassisSparkID = 3;
    public static final int f_rightChassisSparkID = 7;
    public static final int elevatorSparkMaxID = 4;
    public static final int armSparkMaxID = 8;
    public static final int wristSparkMaxID = 10;
    public static final int m_intakerSparkID = 1;
    public static final int f_intakerSparkID = 9;
    //MOTORS LIMIT SWITCH
    public static final int armMotorLimitSwitchID = 0;
  }
  
  //***PID VALUES***
  public static class PIDValues {
    public static final Distance wheelDiameter = Inches.of(3);
    public static final double elevatorGearRatio = 36;
    public static final double armGearRatio = 48;
    public static final double wristGearRatio = 36;
    public static final Distance distancePerRotation = wheelDiameter.times(Math.PI).divide(wristGearRatio);
    public static final Distance kelevatorLength = Inches.of(32.25);
       //Chassis
       public static final double chassisP = 0.1;
       public static final double chassisI = 0;
       public static final double chassisD = 7.5;
       public static final double chassisMin = -0.25;
       public static final double chassisMax = 0.25;
    //Elevator Slot0
    public static final double elevatorP0 = 1;
    public static final double elevatorI0 = 0;
    public static final double elevatorD0 = 12.5;
    public static final double elevatorMin0 = -0.75;
    public static final double elevatorMax0 = 1;
    //Elevator Slot1
    public static final double elevatorP1 = 2;
    public static final double elevatorI1 = 0;
    public static final double elevatorD1 = 0;
    public static final double elevatorMin1 = -0.5;
    public static final double elevatorMax1 = 1;
   //Arm
    public static final double armP = 0.25;
    public static final double armI = 0;
    public static final double armD = 7.5;
    public static final double armMin = -0.125;
    public static final double armMax = 0.35;
   //Arm Slot1
    public static final double armP1 = 0.25;
    public static final double armI1 = 0;
    public static final double armD1 = 12.5;
    public static final double armMin1 = -0.05;
    public static final double armMax1 = 0.5;
   //Arm Slot2
    public static final double armP2 = 0.2;
    public static final double armI2 = 0;
    public static final double armD2 = 7.5;
    public static final double armMin2 = -0.2;
    public static final double armMax2 = 0.65;
    //Wrist
    public static final double wristP = 0.1;
    public static final double wristI = 0;
    public static final double wristD = 7.5;
    public static final double wristMin = -0.4;
    public static final double wristMax = 0.125;
    //Wrist Slot 1
    public static final double wristP1 = 0.2;
    public static final double wristI1 = 0;
    public static final double wristD1 = 7.5;
    public static final double wristMin1 = -0.05;
    public static final double wristMax1 = 0.225;
    }

  //***SOLENOID'S ID***
  public static class PneumaticsdId{
    public static final int transmissionFwdID = 0;
    public static final int transmissionRvsID = 1;
    
}

  //***MOTORS CONFIG PARAMETERs***
  public static class MotorsConfig{
    public static final int curentLimit = 40;
    public static final double rampRate = 1;
    public static final double Tdelay = 0.5;
  }

  //***MISCELLANEOUS PARAMETERS***
  public static class Miscellaneous{
    public static final double tDelay = 0.5;
    public static final int elevatorLimitSwitchID = 0;
  }
}




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Miscellaneous;
import frc.robot.Constants.MotorsConfig;
import frc.robot.Constants.MotorsId;
import frc.robot.Constants.PIDValues;
import frc.robot.Constants.PneumaticsdId;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  //***CONTROLLERS***
  private Joystick driverController;
  private Joystick operatorController;

  //***CHASSIS SUBSYSTEM***
  private SparkMax m_leftChassisSpark;
  private SparkMax m_rightChassisSpark;

  private SparkMax f_leftChassisSpark;
  private SparkMax f_rightChassisSpark;

  private SparkMaxConfig m_leftChassisConfig;    
  private SparkMaxConfig m_rightChassisConfig;

  private SparkMaxConfig f_leftChassisConfig;    
  private SparkMaxConfig f_rightChassisConfig;

  private RelativeEncoder m_leftChassisEncoder;
  private RelativeEncoder m_rightChassisEncoder;

  private SparkClosedLoopController m_leftChassisClosedLoopController;
  private SparkClosedLoopController m_rightChassisClosedLoopController;


  private DifferentialDrive chassis;

  private SlewRateLimiter drivingFilter;
   
   //***ELEVATOR SUBSYSTEM***
    private SparkMax elevatorMotor;
    private SparkMaxConfig elevatorMotorConfig;
    private SparkClosedLoopController elevatorClosedLoopController;
    private RelativeEncoder elevatorEncoder;
    private DigitalInput elevatorLimitSwitch;

   
    //***ARM SUBSYSTEM***
    private SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private SparkClosedLoopController armClosedLoopController;
    private RelativeEncoder armEncoder;
    //private DigitalInput armLimitSwtich;

    //***WRIST SUBSYSTEM***
    private SparkMax wristMotor;
    private SparkMaxConfig wristMotorConfig;
    private SparkClosedLoopController wristClosedLoopController;
    private RelativeEncoder wristEncoder;
    //private DigitalInput wristLimitSwtich;


    //***INTAKER SUBSYSTEM***
    private SparkMax m_intakerMotor;
    private SparkMaxConfig m_intakerMotorConfig;

    private SparkMax f_intakerMotor;
    private SparkMaxConfig f_intakerMotorConfig;

    //***PNEUMATICS***
    private Compressor compressor;
    private DoubleSolenoid transmissionSolenoid;

    //***MISCELLANEUOUS***
    private UsbCamera camera1;
    private UsbCamera camera2;

  public Robot() {
    
    //***CONTROLLERS***
    driverController = new Joystick(ControllerConstants.driverControllerID);
    operatorController = new Joystick(ControllerConstants.operatorControllerID);

    //***CHASSIS SUBSYSTEM***
    m_leftChassisSpark = new SparkMax(MotorsId.m_leftChassisSparkID, MotorType.kBrushless);
    m_rightChassisSpark = new SparkMax(MotorsId.m_rightChassisSparkID, MotorType.kBrushless);

    f_leftChassisSpark = new SparkMax(MotorsId.f_leftChassisSparkID, MotorType.kBrushless);
    f_rightChassisSpark = new SparkMax(MotorsId.f_rightChassisSparkID, MotorType.kBrushless);

    m_leftChassisConfig = new SparkMaxConfig();
    m_rightChassisConfig = new SparkMaxConfig();

    f_leftChassisConfig = new SparkMaxConfig();
    f_rightChassisConfig = new SparkMaxConfig();

    m_leftChassisClosedLoopController = m_leftChassisSpark.getClosedLoopController();
    m_leftChassisEncoder = m_leftChassisSpark.getEncoder();

    m_rightChassisClosedLoopController = m_rightChassisSpark.getClosedLoopController();
    m_rightChassisEncoder = m_rightChassisSpark.getEncoder();

    drivingFilter = new SlewRateLimiter(6);

    m_leftChassisConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(MotorsConfig.rampRate)
      .closedLoopRampRate(MotorsConfig.rampRate)
      .smartCurrentLimit(MotorsConfig.curentLimit)
      .disableFollowerMode();

      m_leftChassisConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(PIDValues.chassisP)
      .i(PIDValues.chassisI)
      .d(PIDValues.chassisD)
      .outputRange(PIDValues.chassisMin, PIDValues.chassisMax);

    m_rightChassisConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(MotorsConfig.rampRate)
      .closedLoopRampRate(MotorsConfig.rampRate)
      .smartCurrentLimit(MotorsConfig.curentLimit)
      .disableFollowerMode();

      m_rightChassisConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(PIDValues.chassisP)
      .i(PIDValues.chassisI)
      .d(PIDValues.chassisD)
      .outputRange(PIDValues.chassisMin, PIDValues.chassisMax);
     
    f_leftChassisConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(MotorsConfig.rampRate)
      .closedLoopRampRate(MotorsConfig.rampRate)
      .smartCurrentLimit(MotorsConfig.curentLimit)
      .follow(MotorsId.m_leftChassisSparkID);

    f_rightChassisConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(MotorsConfig.rampRate)
      .closedLoopRampRate(MotorsConfig.rampRate)
      .smartCurrentLimit(MotorsConfig.curentLimit)
      .follow(MotorsId.m_rightChassisSparkID);

      m_leftChassisSpark.configure(m_leftChassisConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_rightChassisSpark.configure(m_rightChassisConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  
      f_leftChassisSpark.configure(f_leftChassisConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      f_rightChassisSpark.configure(f_rightChassisConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Chassis Left Target Position", 0);
    SmartDashboard.setDefaultBoolean("Chassis Left Reset Encoder", false);

    SmartDashboard.setDefaultNumber("Chassis Right Target Position", 0);
    SmartDashboard.setDefaultBoolean("Chassis Right Reset Encoder", false);

    //***ELEVATOR SUBSYSTEM***
    elevatorMotor = new SparkMax(MotorsId.elevatorSparkMaxID, MotorType.kBrushless);
    elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorLimitSwitch = new DigitalInput(Miscellaneous.elevatorLimitSwitchID);

    elevatorMotorConfig = new SparkMaxConfig();
    
    elevatorMotorConfig 
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0)
      .closedLoopRampRate(0)
      .smartCurrentLimit(40)
      .disableFollowerMode();

    elevatorMotorConfig.encoder
      .positionConversionFactor(1);

    elevatorMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(PIDValues.elevatorP0)
      .i(PIDValues.elevatorI0)
      .d(PIDValues.elevatorD0)
      .outputRange(PIDValues.elevatorMin0, PIDValues.elevatorMax0)
      // Set PID gains for velocity control in slot 1
      .p(PIDValues.elevatorP1, ClosedLoopSlot.kSlot1)
      .i(PIDValues.elevatorI1, ClosedLoopSlot.kSlot1)
      .d(PIDValues.elevatorD1, ClosedLoopSlot.kSlot1)
      .outputRange(PIDValues.elevatorMin1, PIDValues.elevatorMax1, ClosedLoopSlot.kSlot1);

    elevatorMotor.configure(elevatorMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Elevator Target Position", 0);
    SmartDashboard.setDefaultBoolean("Elevator Reset Encoder", false);

    
   //***ARM SUBSYSTEM***
   armMotor = new SparkMax(MotorsId.armSparkMaxID, MotorType.kBrushless);
   armClosedLoopController = armMotor.getClosedLoopController();
   armEncoder = armMotor.getEncoder();

   armMotorConfig = new SparkMaxConfig();
   
   armMotorConfig 
     .inverted(false)
     .idleMode(IdleMode.kBrake)
     .openLoopRampRate(0)
     .closedLoopRampRate(0)
     .smartCurrentLimit(40)
     .disableFollowerMode();

   armMotorConfig.encoder
     .positionConversionFactor(1);

   armMotorConfig.closedLoop
     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
     // Set PID values for position control. We don't need to pass a closed loop
     // slot, as it will default to slot 0.
     .p(PIDValues.armP)
     .i(PIDValues.armI)
     .d(PIDValues.armD)
     .outputRange(PIDValues.armMin, PIDValues.armMax)
      // Set PID gains for velocity control in slot 1
     .p(PIDValues.armP1, ClosedLoopSlot.kSlot1)
     .i(PIDValues.armI1, ClosedLoopSlot.kSlot1)
     .d(PIDValues.armD1, ClosedLoopSlot.kSlot1)
     .outputRange(PIDValues.armMin1, PIDValues.armMax1, ClosedLoopSlot.kSlot1)
      // Set PID gains for velocity control in slot 2
     .p(PIDValues.armP2, ClosedLoopSlot.kSlot2)
     .i(PIDValues.armI2, ClosedLoopSlot.kSlot2)
     .d(PIDValues.armD2, ClosedLoopSlot.kSlot2)
     .outputRange(PIDValues.armMin2, PIDValues.armMax2, ClosedLoopSlot.kSlot2);

     armMotor.configure(armMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
     
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Arm Target Position", 0);
    SmartDashboard.setDefaultBoolean("Arm Reset Encoder", false);
    

    //***WRIST SUBSYSTEM***
    wristMotor = new SparkMax(MotorsId.wristSparkMaxID, MotorType.kBrushless);
    wristClosedLoopController = wristMotor.getClosedLoopController();
    wristEncoder = wristMotor.getEncoder();

    wristMotorConfig = new SparkMaxConfig();
    
    wristMotorConfig 
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0)
      .closedLoopRampRate(0)
      .smartCurrentLimit(40)
      .disableFollowerMode();

    wristMotorConfig.encoder
      .positionConversionFactor(1);

    wristMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(PIDValues.wristP)
      .i(PIDValues.wristI)
      .d(PIDValues.wristD)
      .outputRange(PIDValues.wristMin, PIDValues.wristMax)
      // Set PID gains for velocity control in slot 1
      .p(PIDValues.wristP1, ClosedLoopSlot.kSlot1)
      .i(PIDValues.wristI1, ClosedLoopSlot.kSlot1)
      .d(PIDValues.wristD1, ClosedLoopSlot.kSlot1)
      .outputRange(PIDValues.wristMin1, PIDValues.wristMax1, ClosedLoopSlot.kSlot1);

      wristMotor.configure(wristMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Wrist Target Position", 0);
    SmartDashboard.setDefaultBoolean("Wrist Reset Encoder", false);


     //***INATKER SUBSYSTEM***
     m_intakerMotor = new SparkMax(MotorsId.m_intakerSparkID, MotorType.kBrushed); 
     m_intakerMotorConfig = new SparkMaxConfig();

     f_intakerMotor = new SparkMax(MotorsId.f_intakerSparkID, MotorType.kBrushed); 
     f_intakerMotorConfig = new SparkMaxConfig();
     
     m_intakerMotorConfig 
       .inverted(false)
       .idleMode(IdleMode.kBrake)
       .openLoopRampRate(0)
       .closedLoopRampRate(0)
       .smartCurrentLimit(40)
       .disableFollowerMode();

       f_intakerMotorConfig 
       .idleMode(IdleMode.kBrake)
       .openLoopRampRate(0)
       .closedLoopRampRate(0)
       .smartCurrentLimit(40)
       .follow(MotorsId.m_intakerSparkID, true);

       m_intakerMotor.configure(m_intakerMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
       f_intakerMotor.configure(f_intakerMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    //***PNEUMATICS***
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableDigital();
    transmissionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsdId.transmissionFwdID, PneumaticsdId.transmissionRvsID);
    transmissionSolenoid.set(DoubleSolenoid.Value.kForward);

    //***MISCELLANEOUS***
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setResolution(160, 120);
    camera1.setFPS(30);

    camera2 = CameraServer.startAutomaticCapture(1);
    camera2.setResolution(160, 120);
    camera2.setFPS(30);

  }

  @SuppressWarnings("PMD.UnconditionalIfStatement")
  @Override
  public void robotPeriodic() {
    

    SmartDashboard.putNumber("Left Actual Position", m_leftChassisEncoder.getPosition());
    SmartDashboard.putNumber("Right Actual Position", m_rightChassisEncoder.getPosition());

    //***ELEVATOR SUBSYSTEM***
     // Display encoder position and velocity
     SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
     SmartDashboard.putBoolean("Elevator LimitSwitch", elevatorLimitSwitch.get());
 
     if (elevatorLimitSwitch.get() == false) {
       elevatorEncoder.setPosition(0);
     }


     //***ARM SUBSYSTEM***
    // Display encoder position and velocity
    SmartDashboard.putNumber("Arm Actual Position", armEncoder.getPosition());
 
    if (SmartDashboard.getBoolean("Arm Reset Encoder", false)) {
      SmartDashboard.putBoolean("Arm Reset Encoder", false);
      // Reset the encoder position to 0
      armEncoder.setPosition(0);
    } 


    //***WRIST SUBSYSTEM***
    // Display encoder position and velocity
     SmartDashboard.putNumber("Wrist Actual Position", wristEncoder.getPosition());
 
     if (SmartDashboard.getBoolean("Wrist Reset Encoder", false)) {
       SmartDashboard.putBoolean("Wrist Reset Encoder", false);
       // Reset the encoder position to 0
       wristEncoder.setPosition(0);
     }
    

  }

  @Override
  public void autonomousInit() {
    
    m_leftChassisEncoder.setPosition(0);
    m_rightChassisEncoder.setPosition(0);

    elevatorClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    wristClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    armClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0); 
    

  
  }

  @Override
  public void autonomousPeriodic() {
    
    m_leftChassisClosedLoopController.setReference(-42.5, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_rightChassisClosedLoopController.setReference(-42.5, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    Timer.delay(4.5);
    elevatorClosedLoopController.setReference((46.5*127.6)/51, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    armClosedLoopController.setReference((20*PIDValues.armGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    if (armEncoder.getPosition() > 0.52) {
      wristClosedLoopController.setReference((140*PIDValues.wristGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  
  }

  @Override
  public void teleopInit() {
    
    chassis = new DifferentialDrive(m_leftChassisSpark, m_rightChassisSpark);

  }

  @Override
  public void teleopPeriodic() {
    
    //***CHASSIS SUBSYSTEM***
    if ((driverController.getRawButton(ControllerConstants.R1) == true) && (transmissionSolenoid.get() == DoubleSolenoid.Value.kForward)){
      m_leftChassisSpark.stopMotor();
      m_rightChassisSpark.stopMotor();
      transmissionSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else if(driverController.getRawButton(ControllerConstants.L1) == true && (transmissionSolenoid.get() == DoubleSolenoid.Value.kReverse)){
        m_leftChassisSpark.stopMotor();
        m_rightChassisSpark.stopMotor();
          transmissionSolenoid.set(DoubleSolenoid.Value.kForward); 
    } else {
        chassis.arcadeDrive(drivingFilter.calculate(driverController.getRawAxis(ControllerConstants.LY)), driverController.getRawAxis(ControllerConstants.RX), true);
    }


    //***ELEVATOR SUBSYSTEM***
    /*
    double elevatorTargetPosition = SmartDashboard.getNumber("Elevator Target Position", 0);
    double elevatorPositionAdjusted = (elevatorTargetPosition*127.6)/51;
    elevatorClosedLoopController.setReference(elevatorPositionAdjusted, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    */

    //***ALGAE REEF PICKUP AND BARGE DEPOSIT***
    //ELEVATOR GROUND LEVEL
    if (operatorController.getRawButton(ControllerConstants.A) || operatorController.getRawButton(ControllerConstants.L3)) {
      elevatorClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      wristClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      armClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);       
    } 
    //ElELATOR ALGAE LEVEL 1
    else if (operatorController.getRawButton(ControllerConstants.B)) {
      elevatorClosedLoopController.setReference((46.5*127.6)/51, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      armClosedLoopController.setReference((20*PIDValues.armGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      transmissionSolenoid.set(DoubleSolenoid.Value.kReverse);
        if (armEncoder.getPosition() > 0.52) {
          wristClosedLoopController.setReference((60*PIDValues.wristGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }

    //ElEVATOR ALGAE LEVEL 2
    }  else if (operatorController.getRawButton(ControllerConstants.X)) {
      armClosedLoopController.setReference((120*PIDValues.armGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      transmissionSolenoid.set(DoubleSolenoid.Value.kReverse);
      if (armEncoder.getPosition() > 5) {
        wristClosedLoopController.setReference((100*PIDValues.wristGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      }

    //ELEVATOR BARGE LEVEL
    } else if (operatorController.getRawButton(ControllerConstants.Y)) {
        armClosedLoopController.setReference((170*PIDValues.armGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot2);
        transmissionSolenoid.set(DoubleSolenoid.Value.kReverse);
      if (armEncoder.getPosition() > 8) {
        elevatorClosedLoopController.setReference((46.5*127.6)/51, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        wristClosedLoopController.setReference((150*PIDValues.wristGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      }
    }

    //***ELEVATOR CLIMBER POSITIONS***
    //ELEVATOR LIFT UP
    if (-operatorController.getRawAxis(ControllerConstants.LY) < -0.2) {
      elevatorClosedLoopController.setReference((5*127.6)/51, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      transmissionSolenoid.set(DoubleSolenoid.Value.kReverse);
    } 
    //ELEVATOR SHALLOW CAGE POSITION
    else if (-operatorController.getRawAxis(ControllerConstants.LY) > 0.2) {
      elevatorClosedLoopController.setReference((30*127.6)/51, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      transmissionSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //***ARM SUBSYSTEM***
/*
    double armTargetPosition = SmartDashboard.getNumber("Arm Target Position", 0);
    double armTargetPositionAdjusted = (armTargetPosition*PIDValues.armGearRatio)/360;
    armClosedLoopController.setReference(armTargetPositionAdjusted, ControlType.kPosition, ClosedLoopSlot.kSlot2);

    //***WRIST SUBSYSTEM***
 
    double wristTargetPosition = SmartDashboard.getNumber("Wrist Target Position", 0);
    double wristTargetPositionAdjusted = (wristTargetPosition*PIDValues.wristGearRatio)/360;
    wristClosedLoopController.setReference(wristTargetPositionAdjusted, ControlType.kPosition, ClosedLoopSlot.kSlot1);
 */  

 //***FLOOR PICKUP & PROCESSOR DEPOSIT
  //WRIST FLOOR PICKUP
  if (-operatorController.getRawAxis(ControllerConstants.RY) < -0.2) {
      armClosedLoopController.setReference((30*PIDValues.armGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      if (armEncoder.getPosition() > 0.5) {
        wristClosedLoopController.setReference((123.5*PIDValues.wristGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
    } 
    //WRIST PROCESSOR DEPOSIT
    else if (-operatorController.getRawAxis(ControllerConstants.RY) > 0.2) {
      armClosedLoopController.setReference((30*PIDValues.armGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      if (armEncoder.getPosition() > 0.5) {
        wristClosedLoopController.setReference((85*PIDValues.wristGearRatio)/360, ControlType.kPosition, ClosedLoopSlot.kSlot0);      
      }
    } 
    //WRIST INITIAL POSITION    
    else if (operatorController.getRawButton(ControllerConstants.R3)) {
      wristClosedLoopController.setReference(0.00, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      if (wristEncoder.getPosition() < 6) {
        armClosedLoopController.setReference(0.00, ControlType.kPosition, ClosedLoopSlot.kSlot0);      
      }     
    }


    //***INTAKER SUBSYSTEM***
    if (operatorController.getRawButton(ControllerConstants.L1)) {
      m_intakerMotor.set(0.75);
    } else if (operatorController.getRawButton(ControllerConstants.R1)) {
      m_intakerMotor.set(-1);
    }  else {
      m_intakerMotor.set(0);
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    elevatorClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    wristClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    armClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0); 
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

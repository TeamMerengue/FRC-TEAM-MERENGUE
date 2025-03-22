// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//Imports from Constant.java
import frc.robot.Constant.ButtonConst;
import frc.robot.Constant.Config;
import frc.robot.Constant.MotorsId;
import frc.robot.Constant.PneumaticsdId;

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

    //***Controllers***
    Joystick driverController = new Joystick(ButtonConst.driverControllerID);
    Joystick opController = new Joystick(ButtonConst.operatorControllerID);
    
    //***Motor Controllers***
    SparkMax m_leftChassisSpark = new SparkMax(MotorsId.m_leftChassisSparkID, MotorType.kBrushless);
    SparkMax m_rightChassisSpark = new SparkMax(MotorsId.m_rightChassisSparkID, MotorType.kBrushless);

    SparkMax f_leftChassisSpark = new SparkMax(MotorsId.f_leftChassisSparkID, MotorType.kBrushless);
    SparkMax f_rightChassisSpark = new SparkMax(MotorsId.f_rightChassisSparkID, MotorType.kBrushless);

    SparkMax m_climberSpark = new SparkMax(MotorsId.m_climberSparkID, MotorType.kBrushless);
    SparkMax f_climberSpark = new SparkMax(MotorsId.f_climberSparkID, MotorType.kBrushless);
    
    SparkMax m_intakerSpark = new SparkMax(MotorsId.m_intakerSparkID, MotorType.kBrushless);
    SparkMax f_intakerSpark = new SparkMax(MotorsId.f_intakerSparkID, MotorType.kBrushless);

    //***Motor Controllers Config***
    SparkMaxConfig m_leftChassisConfig = new SparkMaxConfig();    
    SparkMaxConfig m_rightChassisConfig = new SparkMaxConfig();

    SparkMaxConfig f_leftChassisConfig = new SparkMaxConfig();    
    SparkMaxConfig f_rightChassisConfig = new SparkMaxConfig();

    SparkMaxConfig m_intakerConfig = new SparkMaxConfig();    
    SparkMaxConfig f_intakerConfig = new SparkMaxConfig();

    SparkMaxConfig m_climberConfig = new SparkMaxConfig();    
    SparkMaxConfig f_climberConfig = new SparkMaxConfig();

    //***Chassis***
    DifferentialDrive chassis = new DifferentialDrive(m_leftChassisSpark, m_rightChassisSpark);

    //***Pneumatics***
    private final DoubleSolenoid wristOpenerSolenoid =
    new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsdId.wristOpenerFwdID, PneumaticsdId.wristOpenerRvsID);

    private final DoubleSolenoid wristRaiserSolenoid =
    new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsdId.wristRaiserFwdID, PneumaticsdId.wristRaiserRvsID);

    private final DoubleSolenoid transmissionSolenoid =
    new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsdId.transmissionFwdID, PneumaticsdId.transmissionRvsID);
    
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    Shuffleboard tab;

  public Robot() {

// Publish elements to shuffleboard //
    ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
    pneumaticsTab.add("Compressor", compressor);
    pneumaticsTab.add("Transmission Solenoid", transmissionSolenoid);
    pneumaticsTab.add("Wrist Opener Solenoid", wristOpenerSolenoid);
    pneumaticsTab.add("Wrist Raiser Solenoid", wristRaiserSolenoid);
    // Also publish some raw data
    // Get the pressure (in PSI) from the analog sensor connected to the PH.
    // This function is supported only on the PH!
    // On a PCM, this function will return 0.
    pneumaticsTab.addDouble("PH Pressure [PSI]", compressor::getPressure);
    // Get compressor current draw.
    pneumaticsTab.addDouble("Compressor Current", compressor::getCurrent);
    // Get whether the compressor is active.
    pneumaticsTab.addBoolean("Compressor Active", compressor::isEnabled);
    // Get the digital pressure switch connected to the PCM/PH.
    // The switch is open when the pressure is over ~120 PSI.
    pneumaticsTab.addBoolean("Pressure Switch", compressor::getPressureSwitchValue);
    
    //***Motor Controller Config Assignment***
    m_leftChassisConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(Config.rampRate)
      .closedLoopRampRate(Config.rampRate)
      .smartCurrentLimit(Config.curLimit)
      .disableFollowerMode();

    m_rightChassisConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(Config.rampRate)
      .closedLoopRampRate(Config.rampRate)
      .smartCurrentLimit(Config.curLimit)
      .disableFollowerMode();
     
    f_leftChassisConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(Config.rampRate)
      .closedLoopRampRate(Config.rampRate)
      .smartCurrentLimit(Config.curLimit)
      .follow(MotorsId.m_leftChassisSparkID);

    f_rightChassisConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(Config.rampRate)
      .closedLoopRampRate(Config.rampRate)
      .smartCurrentLimit(Config.curLimit)
      .follow(MotorsId.m_rightChassisSparkID);

      m_intakerConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Config.curLimit);
     
      f_intakerConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Config.curLimit)
      .follow(MotorsId.m_intakerSparkID);
      
// Climber configure // 
      m_climberConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Config.curLimit);
     
      f_climberConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Config.curLimit)
      .follow(MotorsId.m_climberSparkID);

// Chassis Config //
    m_leftChassisSpark.configure(m_leftChassisConfig, null, null);
    m_rightChassisSpark.configure(m_rightChassisConfig, null, null);

    f_leftChassisSpark.configure(f_leftChassisConfig, null, null);
    f_rightChassisSpark.configure(f_rightChassisConfig, null, null);

    m_intakerSpark.configure(m_intakerConfig, null, null);
    f_intakerSpark.configure(f_intakerConfig, null, null);

//Climber Config //
    m_climberSpark.configure(m_intakerConfig, null, null);
    f_climberSpark.configure(f_intakerConfig, null, null);

//Pneumatics Starting Position //
    transmissionSolenoid.set(Value.kReverse);
    wristOpenerSolenoid.set(Value.kReverse);
    wristRaiserSolenoid.set(Value.kReverse);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

//DriveTrain SubSystem //
    if ((driverController.getRawButton(ButtonConst.R1) == true) && (transmissionSolenoid.get() == Value.kForward)){
      m_leftChassisSpark.stopMotor();
      m_rightChassisSpark.stopMotor();
      Timer.delay(Config.Tdelay);
      transmissionSolenoid.set(Value.kReverse);
    }
      else if(driverController.getRawButton(ButtonConst.L1) == true && (transmissionSolenoid.get() == Value.kReverse)){
        m_leftChassisSpark.stopMotor();
        m_rightChassisSpark.stopMotor();
        Timer.delay(Config.Tdelay);
        transmissionSolenoid.set(Value.kForward);
    } else {
        chassis.arcadeDrive(driverController.getRawAxis(ButtonConst.LY), driverController.getRawAxis(ButtonConst.RX), true);
    }
    
//Wrist SubSystem //
    if (opController.getRawButton(ButtonConst.R1)) {
    wristOpenerSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else if (opController.getRawButtonPressed(ButtonConst.L1)) {
    wristOpenerSolenoid.set(DoubleSolenoid.Value.kForward);
   }
      
    if (opController.getRawButton(ButtonConst.X)) {
    wristRaiserSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else if (opController.getRawButtonPressed(ButtonConst.Y)) {
    wristRaiserSolenoid.set(DoubleSolenoid.Value.kForward);
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

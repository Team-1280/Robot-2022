// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ColorSensor;

//import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

//import com.revrobotics.ColorSensorV3;

 

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {

  //private final I2C.Port i2cPort = I2C.Port.kOnboard;

  //ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);


  /*

  MotorController m_frontLeft = new PWMVictorSPX(1);
  MotorController m_rearLeft = new PWMVictorSPX(2);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  MotorController m_frontRight = new PWMVictorSPX(3);
  MotorController m_rearRight = new PWMVictorSPX(4);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  */

  
  //TalonSRX m_leftShooter = new TalonSRX(15);
  //TalonSRX m_rightShooter = new TalonSRX(14);

  // private final PWMTalonSRX Constants.Objects.Motors.m_leftDrive = new PWMTalonSRX(0);
  // private final PWMTalonSRX Constants.Objects.Motors.m_rightDrive = new PWMTalonSRX(2);
  
  
  private final Joystick m_stick = new Joystick(0);
  private final Joystick t_stick = new Joystick(1);

  private final XboxController xboxController = new XboxController(2);
  
  


  private final VictorSPX m_victorIntake = new VictorSPX(11);

  private final Spark m_motor12 = new Spark(3);

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    Constants.Objects.Motors.m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    Constants.m_timer.reset();
    Constants.m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    System.out.println(Constants.Objects.Input.limelight.getv());

    if(xboxController.getBButton()){
      TalonSRX m_leftDrive = new TalonSRX(2);
      TalonSRX m_rightDrive = new TalonSRX(1);
  
      System.out.println("B button pressed");
      m_leftDrive.set(ControlMode.PercentOutput, 0);
      m_rightDrive.set(ControlMode.PercentOutput, 0);
  
    }

    //ColorSensor color = new ColorSensor();
    //color.teleopPeriodic();
    //System.out.println(colorSensor.getColor());
    //System.out.println("(" + colorSensor.getRed() + ", " + colorSensor.getGreen() + ", " + colorSensor.getBlue() + ")");

    // System.out.println(Constants.Objects.Input.limelight.getConstants.Objects.Input.limelightDistance());

    if ( xboxController.getAButton() ) { Constants.Objects.Input.limelight.aim(); }
    

    //Shooter xboxShooter = new Shooter();
    //xboxShooter.teleopPeriodic();

    //Drive sticks = new Drive();  
    //sticks.teleopPeriodic();

    
    //double stick0 = m_stick.getRawAxis(1);
    //double stick1 = t_stick.getRawAxis(1);
    //double XboxController = m_rTrigger.getRightTriggerAxis();
    
      //Constants.Objects.Motors.m_rightDrive.set(ControlMode.PercentOutput, stick0);
      //Constants.Objects.Motors.m_leftDrive.set(ControlMode.PercentOutput, stick1);
    

    /* if(xboxController.getLeftTriggerAxis() != 0){
      m_leftShooter.set(ControlMode.PercentOutput, 1);
      m_rightShooter.set(ControlMode.PercentOutput, -1);
      //m_motor12.setVoltage(1);
      m_victorOuttake.set(ControlMode.PercentOutput, -0.7);
      m_victorIntake.set(ControlMode.PercentOutput, 0.7);
      m_motor12.set(0.7);
      System.out.println("Motor 12 should be done");
    }else if(xboxController.getRightTriggerAxis() != 0){
      m_leftShooter.set(ControlMode.PercentOutput, -1 * 0);
      m_rightShooter.set(ControlMode.PercentOutput, 0); 
      //m_motor12.setVoltage(1);
      m_victorOuttake.set(ControlMode.PercentOutput, 0);
      m_victorIntake.set(ControlMode.PercentOutput, 0);
      m_motor12.set(0);
      System.out.println("Motor 12 should be done");
    } */

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void motorReset(){
    if(xboxController.getBButton()){
      TalonSRX m_leftDrive = new TalonSRX(2);
      TalonSRX m_rightDrive = new TalonSRX(1);
  
      System.out.println("B button pressed");
      m_leftDrive.set(ControlMode.PercentOutput, 0);
      m_rightDrive.set(ControlMode.PercentOutput, 0);
  
    }
  }
}




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;

import java.io.Console;
import java.sql.Driver;
import java.util.Currency;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.opencv.core.TickMeter;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.PS4Controller;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {
  WPI_TalonSRX Talon1 = new WPI_TalonSRX(1);
  WPI_TalonSRX Talon3 = new WPI_TalonSRX(3);
  MotorControllerGroup m_left = new MotorControllerGroup(Talon1, Talon3);
  
  WPI_TalonSRX Talon2 = new WPI_TalonSRX(2);


  WPI_TalonSRX Talon4 = new WPI_TalonSRX(4);
  MotorControllerGroup m_right = new MotorControllerGroup(Talon2, Talon4);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  CANSparkMax m_liftmotor = new CANSparkMax(7, MotorType.kBrushless);
  //CANSparkMax m_arm = new CANSparkMax(6, MotorType.kBrushless);
  //CANSparkMax m_secondlift = new CANSparkMax(7, MotorType.kBrushless);
  
  CANSparkMax m_balls = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax m_otherLift = new CANSparkMax(8, MotorType.kBrushless);

  private final PS4Controller m_stick = new PS4Controller(0);
  //private final XboxController m_stick2 = new XboxController(1);
  private final Timer m_timer = new Timer(); 
  
  //AHRS ahrs = new AHRS(SPI.Port.kMXP);
  double tCP = 1.3279;
  double tCI = 0.0;
  double tCD = 0.0;
  PIDController turnController = new PIDController(tCP, tCI, tCD);
  
  //Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  //DoubleSolenoid Solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Talon1.configFactoryDefault();
    Talon2.configFactoryDefault();
    Talon3.configFactoryDefault();
    Talon4.configFactoryDefault();
    Talon1.setInverted(false);
    CameraServer.startAutomaticCapture();
    turnController.enableContinuousInput(-180.0f, 180.0f);
    SmartDashboard.putNumber("TurnController P", tCP);
    SmartDashboard.putNumber("TurnController I", tCI);
    SmartDashboard.putNumber("TurnController D", tCD);
  }

  @Override
  public void robotPeriodic() {
    turnController.setP(SmartDashboard.getNumber("TurnController P", tCP));
    turnController.setI(SmartDashboard.getNumber("TurnController I", tCI));
    turnController.setD(SmartDashboard.getNumber("TurnController D", tCD));
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    Talon1.setInverted(true);
    Talon3.setInverted(true);
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_drive.arcadeDrive(0.3, 0);
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    Talon1.setInverted(true);
    Talon3.setInverted(true);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    m_liftmotor.set(m_stick.getRightY());
    
    if (m_stick.getSquareButton()) {
      m_balls.set(m_stick.getR2Axis());
    } else {
      m_balls.set(-m_stick.getR2Axis());
    }

    if (m_stick.getCircleButton()) {
      m_otherLift.set(m_stick.getR2Axis());
    } else {
      m_otherLift.set(-m_stick.getR2Axis());
    }

    m_drive.arcadeDrive(m_stick.getLeftY(), -(m_stick.getLeftX()*0.5));
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

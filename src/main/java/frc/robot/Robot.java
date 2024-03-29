/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import frc.robot.Drive_Train;

//import edu.wpi.first.wpilibj.I2C;
//import java.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;

//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.vision.VisionRunner;
//import edu.wpi.first.wpilibj.vision.VisionThread;
///import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Scheduler;
//import frc.robot.OI;
//import java.lang.Math.*;
/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
// import com.ctre.phoenix.motorcontrol.ControlMode;

public class Robot extends TimedRobot {
  int runs;
  public static PWM light = new PWM(0);
  /*
   * UsbCamera camera1 = new UsbCamera("front?", 1); UsbCamera camera2;
   */
  UsbCamera camera1;
  UsbCamera camera2;

  boolean circle = false;
  boolean insane = false;
  NetworkTableEntry cameraSelection;
  // VideoSink server;
  boolean ZUC = true;
  boolean buttonimp = false;
  double FF = 0;
  static double kP = 0.000060;
  static double kFF = 0.000173;
  // trap and auton stuff
  TrapezoidalMove trap = new TrapezoidalMove();
  Autonomous auto = new Autonomous();

  @Override
  public void robotInit() {
    // start dual camera (OUTDATED. USE LIMELIGHT)

    FinalShooter.MidRange();
    SmartDashboard.putNumber("P", FinalShooter.kP);

    SmartDashboard.putNumber("FF", FinalShooter.kFF);

  }

  @Override
  public void robotPeriodic() {
    
  }
/*
SmartDashboard.putNumber("lime", (double) NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").getNumber(5));
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
*/
  @Override
  public void teleopInit() {
    Autonomous.autonInit();
    FinalShooter.FinalSHTInit();

    FinalShooter.shuteInit();
    Drive_Train.DriveAndrew();
    FileLogger.createFile("test");
  }

  @Override
  public void teleopPeriodic() {
    // Shooter prints
    SmartDashboard.putNumber("set top", FinalShooter.topvelocity);
    SmartDashboard.putNumber("set bottom", FinalShooter.bottomvelocity);
    OI.OIUpdate();
    Intake.runIntake();
    Limelight.Target();
    Drive_Train.drive();
    FinalShooter.trauma();
    Climber.Climb();
    
    Autonomous.learnMode();
    
  }

  @Override
  public void disabledInit() {
    // resets
    Autonomous.autonDis();
    Drive_Train.RobotDis();
    Intake.intakeDis();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    // FileLogger.createFile("learn_mode");
    // Drive_Train.LeftMotorEncoder.setPosition(0);
    // Drive_Train.RightMotorEncoder.setPosition(0);
    Autonomous.autonInit();

    // Drive_Train.RightMotor.setIdleMode(IdleMode.kCoast);
    // Drive_Train.LeftMotor.setIdleMode(IdleMode.kCoast);
    // Autonomous.timerForward.start();
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println(Limelight.ballClose());
    System.out.println("ea:" + Limelight.ea);
    Drive_Train.RightMotor.setIdleMode(IdleMode.kCoast);
    Drive_Train.LeftMotor.setIdleMode(IdleMode.kCoast);
    //Autonomous.circlePID(3, Math.PI/2, 0.25, 0, false, true);
    //Autonomous.MovePID(2);
    //Autonomous.learnMode();
    //Autonomous.chainFunction();
}
}

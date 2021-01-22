/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

//import frc.robot.Drive_Train;

//import edu.wpi.first.wpilibj.I2C;
//import java.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  /*UsbCamera camera1 = new UsbCamera("front?", 1);
  UsbCamera camera2;*/
  UsbCamera camera1;
  UsbCamera camera2;
  
  boolean circle = false;
  boolean insane = false;
  NetworkTableEntry cameraSelection;
  //VideoSink server;
  boolean ZUC = true;
  boolean buttonimp = false;
  double FF = 0;
  static double kP =0.000060;
  static double kFF = 0.000173;
//trap and auton stuff
  TrapezoidalMove trap = new TrapezoidalMove();
  Autonomous auto = new Autonomous();

  @Override
  public void robotInit() {
    //start dual camera (OUTDATED. USE LIMELIGHT)
    
    FinalShooter.MidRange();
    SmartDashboard.putNumber("P", FinalShooter.kP);
    
    SmartDashboard.putNumber("FF", FinalShooter.kFF);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopInit() {
    FinalShooter.FinalSHTInit();

    FinalShooter.shuteInit();
    Drive_Train.DriveAndrew();
    
    
  }

  @Override
  public void teleopPeriodic() {
    //Shooter prints
    SmartDashboard.putNumber("set top", FinalShooter.topvelocity);
    SmartDashboard.putNumber("set bottom", FinalShooter.bottomvelocity);

    Intake.runIntake();
    Limelight.Target();
    Drive_Train.drive();
    FinalShooter.trauma();
    Climber.Climb();
    //OUTDATED CAMERA CODE. Delete when updated with LimeLight.
    if (OI.Right_Joystick.getRawButtonPressed(7)) {
      System.out.println("Setting camera 2");
      cameraSelection.setString(camera2.getName());
  } else if (OI.Right_Joystick.getRawButtonReleased(7)) {
      System.out.println("Setting camera 1");
      cameraSelection.setString(camera1.getName());
  }
  }


  @Override
  public void disabledInit() {
    //resets
    Drive_Train.RobotDis();

    Intake.intakeDis();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    Autonomous.stracker=0;
    Autonomous.first =true;
    SmartDashboard.putNumber("dP", 0);
    SmartDashboard.putNumber("position", 0);
    Autonomous.timerForward.reset();
    Drive_Train.RightMotor.restoreFactoryDefaults();
    Drive_Train.DriveInit();
    Intake.Soubway.set(ControlMode.PercentOutput, 0);
    
    Drive_Train.RightMotor.setInverted(false);
    Autonomous.autoTracker = 0;
    Autonomous.shootTracker = 0;
    Autonomous.routineTracker = 0;
    Drive_Train.LeftMotorEncoder.setPosition(0);
    Drive_Train.RightMotorEncoder.setPosition(0);
    System.out.println(Drive_Train.RightMotorEncoder.getPosition());
    Drive_Train.RightMotorEncoder.setPositionConversionFactor(Autonomous.conversionFactor);
    Drive_Train.LeftMotorEncoder.setPositionConversionFactor(Autonomous.conversionFactor);

    Autonomous.arctracker = 0;
    //trap.SetAll(40, 60,60, 55);
  } 

  @Override
  public void autonomousPeriodic() {
    //Autonomous.s_drive(0.2,1);
    Autonomous.PFFDriveStraight(0.005, 0.15, 1);
    
    /**I commented out test stuff for now. No auton functions currently set to run. 
     * 
     * Default is OldImprovedAutonPID I believe.
     * 
    //Testing robot wheels
    Drive_Train.LeftMotor.set(0.5);
    Drive_Train.RightMotor.set(0.5);
    SmartDashboard.putNumber("SM1V", FinalShooter.ShooterOneEncoder.getVelocity());
    SmartDashboard.putNumber("SM2V", FinalShooter.ShooterTwoEncoder.getVelocity());
    // Autonomous.drivefromInit(0.3, 12, 3);
    System.out.println("ShootTime: " +Autonomous.timerShooter.get());
    System.out.println("Position: " + trap.Position(Autonomous.timerForward.get()));
    SmartDashboard.putNumber("LeftDrive Power",Drive_Train.RightMotor.get());
    SmartDashboard.putNumber("RightDrive POwer", Drive_Train.LeftMotor.get());
    //Autonomous.ImprovedAutonPID(-trap.Position(Autonomous.timerForward.get()));
    **/
}
}


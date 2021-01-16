/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
Author: Aidan Bradley and Aidan Bradley                                                                                                                                                                                                                                                                                                                                                                    and Raven St.Clair
Why: IDK
*/
package frc.robot;

import java.lang.Math;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.Encoder;

public class Autonomous {
    static double initialPos; // Initial position of robot in auton
    static Timer timerForward = new Timer(); // timer for starting auton shoot routine and trapmove
    static Timer timerShooter = new Timer();// Timer for shooter function
    boolean auto;

    // arguments for switch code.
    static int autoTracker = 0;
    static int shootTracker = 0;
    static int routineTracker = 0;
    static int spinTracker = 0;
    static int stracker = 0;
    // more variables
    static final double trapPositon = 65;
    boolean timerForwardStarted = false;
    // pid variables for shooter
    static double p;
    static double i;
    static double d;
    static double ff;
    // Shooter velocity variables
    static double rpm;
    static double speedTop = 3500;
    static double speedBottom = 5500;
    static boolean first = true;
    static boolean firstDrive = true;


    static TrapezoidalMove trap = new TrapezoidalMove();

    // basic "code block" functions to save time repeatedly typing code.

    /**
     * 
     * @param speed1 power to left motor
     * @param speed2 power to right motor
     */
    public static void autoDrive(double speed1, double speed2) {
        // Basic function that sets wheel speeds in autonomous
        Drive_Train.LeftMotor.set(speed1);
        Drive_Train.RightMotor.set(speed2);
    }

    public static void stopDriving() {
        // stops both wheels
        Drive_Train.LeftMotor.set(0);
        Drive_Train.RightMotor.set(0);
    }

    /**
     * 
     * @param speed    speed at which to travel (-1,1)
     * @param distance distance to feed encoder
     */
    public static void calibrationDrive(double speed, double distance) {
        // For determining the conversion factor for the drive wheel encoders.
        if (Drive_Train.LeftMotorEncoder.getPosition() < distance) {
            autoDrive(speed, speed);
        } else {
            stopDriving();
        }
    }

    // Use kP and kFF for proportional and differential. Default speed is 4200 RPM.
    public static void PIDShooter(double P, double I, double D, double FF, double speedtop, double speedbottom) {
        FinalShooter.ShooterMotorTop.getPIDController().setFF(FF);
        FinalShooter.ShooterMotorTop.getPIDController().setP(P);
        FinalShooter.ShooterMotorTop.getPIDController().setD(D);
        FinalShooter.ShooterMotorBottom.getPIDController().setFF(FF);
        FinalShooter.ShooterMotorBottom.getPIDController().setP(P);
        FinalShooter.ShooterMotorBottom.getPIDController().setD(D);
        FinalShooter.ShooterMotorTop.getPIDController().setReference(speedtop, ControlType.kVelocity);
        FinalShooter.ShooterMotorBottom.getPIDController().setReference(speedbottom, ControlType.kVelocity);
    }

    // PID Drive for Drive_Train in auto
    public static void PFFDriveStraight(double P, double dff, double position) {
        // Drive Train Conversion Factors in inches
        // Gearbox ratio = 10.71
        PFFDriveLeft(P, dff, position);
        PFFDriveRight(P, dff, position);
    }

    public static void PFFDriveSpin(double P, double dff, double position) {
        // Drive Train Conversion Factors in inches
        // Gearbox ratio = 10.71
        PFFDriveLeft(P, dff, position);
        PFFDriveRight(P, dff, -position);
    }

    public static void PFFDriveLeft(double P, double dff, double position) {
        // Drive Train Conversion Factors in inches
        // Gearbox ratio = 10.71
        Drive_Train.LeftMotorEncoder.setPositionConversionFactor(1.7616);
        Drive_Train.leftDrivePID.setFF(dff);
        Drive_Train.leftDrivePID.setP(P);
        Drive_Train.leftDrivePID.setReference(position, ControlType.kPosition);
    }

    public static void PFFDriveRight(double P, double dff, double position) {
        // Drive Train Conversion Factors in inches
        // Gearbox ratio = 10.71
        Drive_Train.RightMotorEncoder.setPositionConversionFactor(1.7616);
        Drive_Train.rightDrivePID.setFF(dff);
        Drive_Train.rightDrivePID.setP(P);
        Drive_Train.rightDrivePID.setReference(position, ControlType.kPosition);

    }

    public static void shootSequence(boolean Auto, double position) {
        // Sequence for shooters. To be used in ALL code, not just auton

        switch (shootTracker) {

            case 0:

                PIDShooter(Robot.kP, 0, 0, Robot.kFF, speedTop, speedBottom);

                if (FinalShooter.ShooterOneEncoder.getVelocity() > speedBottom - 2000
                        && FinalShooter.ShooterTwoEncoder.getVelocity() > speedTop - 2000) {
                    timerShooter.stop();
                    timerShooter.reset();
                    timerShooter.start();
                    shootTracker++;
                }
                break;
            case 1:
                PIDShooter(Robot.kP, 0, 0, Robot.kFF, speedTop, speedBottom);
                if (timerShooter.get() > 0.5) {
                    shootTracker++;
                }
            case 2:
                PIDShooter(Robot.kP, 0, 0, Robot.kFF, speedTop, speedBottom);

                Intake.Indexer.set(ControlMode.PercentOutput, RobotMap.INDEX_MOTOR_VALUE);
                if (timerShooter.get() > 1) {
                    shootTracker++;
                }

                break;
            case 3:
                PIDShooter(Robot.kP, 0, 0, Robot.kFF, speedTop, speedBottom);
                Intake.Soubway.set(ControlMode.PercentOutput, -RobotMap.BELT_MOTOR_VALUE);
                if (timerShooter.get() > 4) {
                    routineTracker = 1;
                }
                break;
        }

    }

    public static void MovePID(double position) {
        // PID movement control for trapezoidal movement.
        SmartDashboard.putNumber("right pos", Drive_Train.RightMotorEncoder.getPosition());
        SmartDashboard.putNumber("left pos", Drive_Train.LeftMotorEncoder.getPosition());

        switch (autoTracker) {
            case 0: // initialize timer.
                initialPos = Drive_Train.RightMotorEncoder.getPosition();
                timerForward.stop();
                timerForward.reset();
                timerForward.start();
                autoTracker++;
                break;
            case 1:
                PFFDriveStraight(0.25, 0, position);

        }

    }

    public static void ImprovedAutonPID(double position) {
        // NEeds to be tested and calibrated before use.
        SmartDashboard.putNumber("right pos", Math.abs(Drive_Train.RightMotorEncoder.getPosition()));
        SmartDashboard.putNumber("left pos", Math.abs(Drive_Train.LeftMotorEncoder.getPosition()));

        switch (routineTracker) {
            case 0: // initialize timer.
                FinalShooter.ShooterMotorTop.setInverted(true);
                System.out.println("Case two");
                spinTracker = 0;
                shootSequence(true, position);

                break;
            case 1:
                // add spin stuff here
                if (spindrive()) {
                    routineTracker = 2;
                }
                break;
            case 2:

                if (firstDrive == true) {
                    trap.SetAll(40, 60, 60, 55);
                    firstDrive = false;
                }
                timerShooter.stop();
                timerShooter.reset();
                initialPos = Drive_Train.RightMotorEncoder.getPosition();
                routineTracker = 3;
                break;
            case 3:

                Robot.light.setRaw(-255);

                System.out.println("Case One");
                // System.out.println("Timer Forward: " + timerForward.get());
                // 5PIDShooter(Robot.kP, 0, 0, Robot.kFF, RobotMap.topvelocity,
                // RobotMap.bottomvelocity);
                MovePID(position);
                if (Math.abs(Drive_Train.RightMotorEncoder.getPosition()) > 49
                        && Math.abs(Drive_Train.LeftMotorEncoder.getPosition()) > 49) {

                    routineTracker = 4;

                }
                break;
            case 4:
                stopDriving();
                // routineTracker++;

                break;

        }

    }

    // post canada
    public static void OldImprovedAutonPID(double position) {
        // NEeds to be tested and calibrated before use.
        SmartDashboard.putNumber("right pos", Math.abs(Drive_Train.RightMotorEncoder.getPosition()));
        SmartDashboard.putNumber("left pos", Math.abs(Drive_Train.LeftMotorEncoder.getPosition()));

        switch (routineTracker) {
            case 0: // initialize timer.
                timerShooter.stop();
                timerShooter.reset();
                initialPos = Drive_Train.RightMotorEncoder.getPosition();
                routineTracker++;

                break;
            case 1:
                Robot.light.setRaw(-255);

                System.out.println("Case One");
                // System.out.println("Timer Forward: " + timerForward.get());
                // 5PIDShooter(Robot.kP, 0, 0, Robot.kFF, RobotMap.topvelocity,
                // RobotMap.bottomvelocity);
                MovePID(position);
                if (Math.abs(Drive_Train.RightMotorEncoder.getPosition()) > 49
                        && Math.abs(Drive_Train.LeftMotorEncoder.getPosition()) > 49) {

                    routineTracker++;

                }

                break;
            case 2:
                stopDriving();
                routineTracker++;
                break;
            case 3:
                FinalShooter.ShooterMotorTop.setInverted(true);
                System.out.println("Case two");

                shootSequence(true, position);
                break;

        }
    }

    // for driving in arcs. As you can see, this does nothing right now.
    public static void arcDrive() {
    }

    public static void s_drive(double position) {
        switch (stracker) {
            case 0:
                Drive_Train.RightMotorEncoder.setPosition(0);
                Drive_Train.LeftMotorEncoder.setPosition(0);
                stracker++;
                break;
            case 1:
                if (((position <0) && (Math.abs(Drive_Train.RightMotorEncoder.getPosition()) < 8.2)) 
                || ((position>0) && (Math.abs(Drive_Train.RightMotorEncoder.getPosition())<8.1))) {
                    Drive_Train.RightMotor.set(position);
                    Drive_Train.LeftMotor.set(position);
                }
                else {
                    stracker++;
                }

                break;
            case 2:
                Drive_Train.RightMotor.set(0);
                Drive_Train.LeftMotor.set(0);
                break;
        }
    }

    public static boolean spindrive() {
        // Function to make robot spin
        switch (spinTracker) {
            case 0: // initialize timer.

                timerShooter.stop();
                timerShooter.reset();
                initialPos = Drive_Train.RightMotorEncoder.getPosition();
                spinTracker = 1;
                break;
            case 1:

                System.out.println("Case One");

                PFFDriveSpin(0.25, 0, Drive_Train.spinDistanceCalculator(30));
                if (Math.abs(Drive_Train.RightMotorEncoder.getPosition()) > -Drive_Train.spinDistanceCalculator(30) + 1
                        && Math.abs(Drive_Train.LeftMotorEncoder.getPosition()) > Drive_Train.spinDistanceCalculator(30)
                                - 1) {

                    spinTracker = 2;

                }
                break;
            case 2:
                stopDriving();

                // routineTracker++;

                break;

        }
        if (spinTracker == 2) {
            return true;
        } else {
            return false;
        }
    }

}
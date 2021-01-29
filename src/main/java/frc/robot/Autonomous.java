/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
Author: Aidan Bradley and Aidan Bradley
            -"Wait, there's two of you?! The horror!" - Raven St. Clair                                                                                                                                                                                                                                                                                                                                                                    and Raven St.Clair
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
    static Timer autonTimer = new Timer(); /*
                                            * general auton timer. Eventually, it may be more practical to use one timer
                                            * and use ti-tf.
                                            */
    static Timer timerForward = new Timer(); // timer for starting auton shoot routine and trapmove
    static Timer timerForward2 = new Timer();
    static Timer timerShooter = new Timer();// Timer for shooter function
    boolean auto;

    // arguments for switch code.
    static int autoTracker = 0;
    static int shootTracker = 0;
    static int routineTracker = 0;
    static int spinTracker = 0;
    static int stracker = 0;
    static int arctracker = 0;
    // more variables
    static final double trapPositon = 65;
    boolean timerForwardStarted = false;
    static double wheelfactor = 6 * Math.PI;
    static double conversionFactor = 1 / ((2 * 72) / (6 * Math.PI));// (1/rev)(6 rev/wheel turn)*(1 wt)/(2*pi*3
                                                                    // inches)*(12 inch/ft))^-1
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
    // arguments for circle drive
    static double d_IN;
    static double d_OUT;
    static double d_RIGHT;
    static double d_LEFT;
    static double arcRatio;
    static TrapezoidalMove trap = new TrapezoidalMove();

    public static void autonInit() {
        // reset trackers and timers and booleans
        Autonomous.stracker = 0;
        Autonomous.autoTracker = 0;
        Autonomous.shootTracker = 0;
        Autonomous.routineTracker = 0;
        Autonomous.arctracker = 0;
        Autonomous.timerForward.reset();
        Autonomous.first = true;

        // reset motor values
        Drive_Train.RightMotor.restoreFactoryDefaults();
        Drive_Train.LeftMotor.restoreFactoryDefaults();
        Drive_Train.DriveInit();
        Drive_Train.RightMotor.setInverted(false);
        Drive_Train.LeftMotor.setInverted(true);
        Drive_Train.LeftMotorEncoder.setPosition(0);
        Drive_Train.RightMotorEncoder.setPosition(0);
        Drive_Train.RightMotorEncoder.setPositionConversionFactor(Autonomous.conversionFactor);
        Drive_Train.LeftMotorEncoder.setPositionConversionFactor(Autonomous.conversionFactor);

        // sets motor follows and idle modes
        Drive_Train.DriveInit();
        
        // Prints

        // add to list if needed
    }

    public static void autonDis() {
        autonTimer.stop();
        autonTimer.reset();
    }

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
        if (Drive_Train.RightMotorEncoder.getPosition() <= position
                || Drive_Train.LeftMotorEncoder.getPosition() <= position) {
            SmartDashboard.putNumber("right pos", Drive_Train.RightMotorEncoder.getPosition());
            SmartDashboard.putNumber("left pos", Drive_Train.LeftMotorEncoder.getPosition());
            PFFDriveLeft(P, dff, position);
            PFFDriveRight(P, dff, position);
        } else {
            stopDriving();
        }
    }

    public static void PFFDriveSpin(double P, double dff, double position) {
        // Drive Train Conversion Factors in inches
        // Gearbox ratio = 10.71
        PFFDriveLeft(P, dff, position);
        PFFDriveRight(P, dff, -position);
    }

    /**
     * 
     * @param P                - Proportional
     * @param dff              - FeedForward
     * @param position         - target displacement
     * @param ConversionFactor - empirically determine
     */
    public static void PFFDriveLeft(double P, double dff, double position) {
        // override to set conversion factor
        Drive_Train.LeftMotorEncoder.setPositionConversionFactor(conversionFactor);
        Drive_Train.leftDrivePID.setFF(dff);
        Drive_Train.leftDrivePID.setP(P);
        Drive_Train.leftDrivePID.setReference(position, ControlType.kPosition);
    }

    public static void PFFDriveRight(double P, double dff, double position) {
        // Drive Train Conversion Factors in inches
        // Gearbox ratio = 10.71
        Drive_Train.RightMotorEncoder.setPositionConversionFactor(conversionFactor);
        Drive_Train.rightDrivePID.setFF(dff);
        Drive_Train.rightDrivePID.setP(P);
        Drive_Train.rightDrivePID.setReference(position, ControlType.kPosition);

    }

    /**
     * calculates arclength
     * 
     * @param midR    - radius of circle traced by center of object
     * @param radDiff difference between center and edges
     * @param theta   angle of arc in RADIANS
     * @return array containing {inner_Displacement, outer_Displacement}
     */
    public static double[] calArcLengths(double midR, double theta) {
        // s = rθ
        double radDiff = Drive_Train.BASE_WIDTH / 24;
        double[] arkLength = {(midR - radDiff) * theta, (midR + radDiff) * theta };
        return arkLength;
    };

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

    public static void s_drive(double power, double distance) {
        switch (stracker) {
            case 0:
                Drive_Train.RightMotorEncoder.setPosition(0);
                Drive_Train.LeftMotorEncoder.setPosition(0);
                stracker++;
                break;
            case 1: // *wheelfactor*distance/24
                if (((power < 0)
                        && (Math.abs(Drive_Train.RightMotorEncoder.getPosition()) < 6 * wheelfactor * distance / 24))
                        || ((power > 0) && (Math.abs(Drive_Train.RightMotorEncoder.getPosition()) < 6 * wheelfactor
                                * distance / 24))) {
                    Drive_Train.RightMotor.set(power);
                    Drive_Train.LeftMotor.set(power);
                    System.out.println(Drive_Train.RightMotorEncoder.getPosition());
                } else {
                    stracker++;
                }

                break;
            case 2:
                Drive_Train.RightMotor.set(0);
                Drive_Train.LeftMotor.set(0);
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
                trap.SetAll(0.5, 0.5, 0.5, position);
                autoTracker++;
                break;
            // any ideas on what this timer actually does? I don't see it used anywhere.
            case 1:
                trap.Position(timerForward.get());
                PFFDriveStraight(0.25, 0, trap.Position(timerForward.get()));
                SmartDashboard.putNumber("expected_positon R", trap.Position(timerForward.get()));

        }

    }

    /**
     * 
     * @param radius    - radius traced by center point
     * @param theta     - angle of arc-segment to be traced by robot
     * @param P         - PID P, try 0
     * @param dFF       - PID feed forward, try something like 0.25 and test other
     *                  values
     * @param clockwise - checks direction we want robot to rotate in.
     */
    public static void circlePID(double radius, double theta, double P, double dFF, boolean clockwise) {
        // try this for circle drive
        SmartDashboard.putNumber("case", arctracker);
        switch (arctracker) {
            case 0:
                Drive_Train.RightMotorEncoder.setPosition(0);
                Drive_Train.LeftMotorEncoder.setPosition(0);
                d_IN = calArcLengths(radius, theta)[0];
                d_OUT = calArcLengths(radius, theta)[1];
                trap.SetAll(1, 5, 3, d_OUT);
                arcRatio = d_IN / d_OUT;
                // loop to check which wheel will travel shortest distance

                startTimers();
                arctracker++;
                break;
            case 1:
                /*
                 * Try changing conversion factor Also, try entering radius, distance, and etc.
                 * in inches and using PFFDrive without the conversion parameter. (I made it
                 * optional).
                 */
                SmartDashboard.putNumber("arcratio", arcRatio);
                SmartDashboard.putNumber("in", d_IN);
                SmartDashboard.putNumber("out", d_OUT);
                SmartDashboard.putNumber("right pos", Drive_Train.RightMotorEncoder.getPosition());
                SmartDashboard.putNumber("left pos", Drive_Train.LeftMotorEncoder.getPosition());
                SmartDashboard.putNumber("expected_positon R", trap.Position(timerForward.get()));
                SmartDashboard.putNumber("expected_positon2 R", trap.Position(timerForward.get()) * arcRatio);

                if (clockwise == true) {
                    d_RIGHT = d_IN;
                    d_LEFT = d_OUT;
                    PFFDriveRight(P, dFF, trap.Position(timerForward.get()) * arcRatio);
                    PFFDriveLeft(P, dFF, trap.Position(timerForward.get()));

                } else {
                    d_RIGHT = d_OUT;
                    d_LEFT = d_IN;
                    PFFDriveRight(P, dFF, trap.Position(timerForward.get()));
                    PFFDriveLeft(P, dFF, trap.Position(timerForward.get()) * arcRatio);
                    ;

                }
                double travledRight = Drive_Train.RightMotorEncoder.getPosition();
                double traveledLeft = Drive_Train.LeftMotorEncoder.getPosition();
                if ((Math.abs(Math.abs(travledRight) - Math.abs(d_RIGHT)) < 0.1)
                        || (Math.abs(Math.abs(traveledLeft) - Math.abs(d_LEFT)) < 0.1)) {
                    arctracker++;
                }
                break;
            case 2:
                stopDriving();
                break;

        }

    }

    public static void startTimers() {
        timerForward.stop();
        timerForward.reset();
        timerForward.start();

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
    public static void learnMode() {
        double[] arguments = { Drive_Train.LeftMotorEncoder.getPosition(), Drive_Train.RightMotorEncoder.getPosition(),
                autonTimer.get() };
        TestOpenFile.writeFile("learn_mode", arguments);
    }

    public static void learnMode(double[] arguments){
        TestOpenFile.writeFile("learn_mode", arguments);
    }
    public static void learnMode(String name, double[] arguments){
        TestOpenFile.writeFile(name, arguments);
    }


}
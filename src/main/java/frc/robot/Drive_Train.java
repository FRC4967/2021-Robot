/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

//import frc.robot.Shooter;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import java.lang.Math.*;
//import frc.robot.SafeMode;

/**
 * Add your docs here.
 */
public class Drive_Train {
    public static CANSparkMax LeftMotor = new CANSparkMax(RobotMap.LEFT_MOTOR_ID, MotorType.kBrushless);
    public static CANSparkMax LeftFollower = new CANSparkMax(RobotMap.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    public static CANSparkMax RightMotor = new CANSparkMax(RobotMap.RIGHT_MOTOR_ID, MotorType.kBrushless);
    public static CANSparkMax RightFollower = new CANSparkMax(RobotMap.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    public static CANEncoder RightMotorEncoder = new CANEncoder(RightMotor);
    public static CANEncoder LeftMotorEncoder = new CANEncoder(LeftMotor);
    public static CANPIDController rightDrivePID = new CANPIDController(RightMotor);
    public static CANPIDController leftDrivePID = new CANPIDController(LeftMotor);
    public static ControlType kVelocity;

    public static void DriveInit() {
        LeftMotor.setIdleMode(IdleMode.kCoast);
        RightMotor.setIdleMode(IdleMode.kCoast);
        RightMotor.setInverted(false);

    }

    public static void DriveAndrew() {
        LeftMotor.setIdleMode(IdleMode.kCoast);
        RightMotor.setIdleMode(IdleMode.kCoast);
        RightFollower.follow(RightMotor);
        LeftFollower.follow(LeftMotor);
        LeftMotor.setInverted(true);
        Intake.Soubway.set(ControlMode.PercentOutput, 0);

    }

    public static void drive() {
        double kP = .0001;
        // double kFF = .001;
        //gets the values of the joysticks
        double rightValue = OI.Right_Joystick.getRawAxis(1);
        double leftValue = OI.Left_Joystick.getRawAxis(1);
        //squares the vaules of the joysticks for better driving control
        double rightSpeed = rightValue * Math.abs(rightValue);
        double leftSpeed = leftValue * Math.abs(leftValue);
        //gets the rpm values of the motors
        double RotationPerMinuteRight = 5650 * rightSpeed;
        double RotationPerMinuteLeft = 5650 * leftSpeed;
        //puts smartdashboard values for the motor velocities and rpm
        SmartDashboard.putNumber("RPM Right", RotationPerMinuteRight);
        SmartDashboard.putNumber("RPM Left", RotationPerMinuteLeft);
        SmartDashboard.putNumber("R", RightMotorEncoder.getVelocity());
        SmartDashboard.putNumber("L", LeftMotorEncoder.getVelocity());
        //puts the values of the joysticks and joysticks squared
        SmartDashboard.putNumber("Right Axis", rightValue);
        SmartDashboard.putNumber("Left Axis", leftValue);
        SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);
        //puts the POV value (the joystick on the top of the joystick)
        SmartDashboard.putNumber("POV", OI.Left_Joystick.getPOV());
        //sets the P value in the PID loop 
        LeftMotor.getPIDController().setP(kP);
        RightMotor.getPIDController().setP(kP);

        if (OI.Left_Joystick.getRawAxis(1) < .15 && -.15 < OI.Left_Joystick.getRawAxis(1)) {
            leftValue = 0;
        }
        if (OI.Right_Joystick.getRawAxis(1) < .15 && -.15 < OI.Right_Joystick.getRawAxis(1)) {
            rightValue = 0;
        }
        //dead zone for the left joystick due to the controllers drifting
        if (leftSpeed > .075) {
            leftValue = OI.Left_Joystick.getRawAxis(1) - .15;
        } else if (OI.Left_Joystick.getRawAxis(1) < -.15) {
            leftValue = OI.Left_Joystick.getRawAxis(1) + .15;
        }
        // dead zone for the right joystick due to the controllers drifting
        if (OI.Right_Joystick.getRawAxis(1) > .15) {
            rightValue = OI.Right_Joystick.getRawAxis(1) - .15;
        } else if (OI.Right_Joystick.getRawAxis(1) < -.15) {
            rightValue = OI.Right_Joystick.getRawAxis(1) + .15;
        }
        if (OI.leftJoystickStraightLock == true) {
            // straight lock for left joysitck
            RightMotor.set(leftSpeed);
            LeftMotor.set(leftSpeed);

        } else if (OI.rightJoystickStraightLock == true) {
            //straight lock on right joystick
            LeftMotor.set(rightSpeed);
            RightMotor.set(rightSpeed);

        } else if (OI.leftJoystickSpinLock == true) {
            //spin lock on the left joystick
            LeftMotor.set(leftSpeed);
            RightMotor.set(leftSpeed * -1);

        } else if (OI.rightJoystickSpinLock == true) {
            //spin lock on right joystick
            LeftMotor.set(rightSpeed * -1);
            RightMotor.set(rightSpeed);

        } else if (OI.Left_Joystick.getRawButton(2)) {
            //inverted drive mode
            LeftMotor.set(leftSpeed * -1);
            RightMotor.set(rightSpeed * -1);
        } else {
            //if no lock buttons are pressed work with respective joysticks
            RightMotor.set(rightSpeed);
            LeftMotor.set(leftSpeed);
            // LeftMotor.set(leftValue);
            // RightMotor.set(rightValue);
        }

    }
    //a function to calculate the spin distance on the wheels in order to rotate a certain amount
    public static double spinDistanceCalculator(double angleOfTurn){
        //circumference = 2 * Pi * radius (radius = wheel width /2)
        //converts the degrees to radians
        double radians = angleOfTurn * Math.PI / 180;
        //calculates arc distance
        double arcDistance = radians * RobotMap.ROBORADIUS;
        return arcDistance;
    }

    public static void RobotDis() {
        //sets the motors to coast when disabled so the wheels arent static and hard to push
        LeftMotor.setIdleMode(IdleMode.kCoast);
        RightMotor.setIdleMode(IdleMode.kCoast);

    }

}
          

    

    
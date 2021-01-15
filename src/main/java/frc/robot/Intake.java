/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//gode
package frc.robot;

//import frc.robot.OI;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.ImpShooter;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.Victor;

public class Intake {
    public static VictorSPX Indexer = new VictorSPX(RobotMap.INDEX_MOTOR_ID);
    public static VictorSPX IntakeLow = new VictorSPX(RobotMap.INTAKEMOTOR2);
    public static VictorSPX IntakeHigh = new VictorSPX(RobotMap.INTAKEMOTOR1);
    public static VictorSPX Soubway = new VictorSPX(RobotMap.BELT_MOTOR_ID);
    public static double wae = 1;

    public static void runIntake() {
        SmartDashboard.putNumber("righty", OI.Right_Joystick.getRawAxis(1) * -1);
        SmartDashboard.putNumber("lefty", OI.Left_Joystick.getRawAxis(1) * -1);
        SmartDashboard.putNumber("high take", IntakeHigh.getMotorOutputPercent());
        SmartDashboard.putNumber("low take", IntakeLow.getMotorOutputPercent());
        SmartDashboard.putString("andrew", "say hi");

        if (OI.intakeRunOut == true) {
            //Button to run intake reversed
            IntakeHigh.set(ControlMode.PercentOutput, -RobotMap.INTAKE_HIGH_TRIGG);
            IntakeLow.set(ControlMode.PercentOutput, -RobotMap.INTAKE_LOW_TRIGG);
        } 
        else {

            if (OI.Left_Joystick.getRawAxis(1) + OI.Right_Joystick.getRawAxis(1) >= 0) {
                //Run low intake by default while robot is in motion.
                IntakeLow.set(ControlMode.PercentOutput, -0.1);
            
                if (OI.Manipulator_Joystick.getRawButton(1) == false) {
                    //High intake set to zero unless manually overriden
                    IntakeHigh.set(ControlMode.PercentOutput, 0);
                }
            } 
            else if (OI.Manipulator_Joystick.getRawButtonReleased(1) == false) {
                //Stops Button when Button is false (released)?
                IntakeHigh.set(ControlMode.PercentOutput, 0);
                IntakeLow.set(ControlMode.PercentOutput, 0);
            }
            
            if(OI.Right_Joystick.getRawButtonReleased(1)){
                //Stops intake when button released
                IntakeHigh.set(ControlMode.PercentOutput, 0);
                IntakeLow.set(ControlMode.PercentOutput, 0);
            }
        }

        if (OI.intakeRunIn == true) {
            //Button to run intake forward
            IntakeHigh.set(ControlMode.PercentOutput, RobotMap.INTAKE_HIGH_TRIGG);
            IntakeLow.set(ControlMode.PercentOutput, RobotMap.INTAKE_LOW_TRIGG);
        } 

    }

    public static void intakeDis() {
        IntakeHigh.set(ControlMode.PercentOutput, 0);
        IntakeLow.set(ControlMode.PercentOutput, 0);
        Indexer.set(ControlMode.PercentOutput, 0);
    }
}
 
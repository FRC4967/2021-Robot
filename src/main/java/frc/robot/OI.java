/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
    public static Joystick Left_Joystick = new Joystick(0);
    public static Joystick Right_Joystick = new Joystick(1);
    public static Joystick Manipulator_Joystick = new Joystick(2);

    public static boolean climberFwdButton = Right_Joystick.getRawButton(9);
    public static boolean climberRevButton = Right_Joystick.getRawButton(6);
    public static boolean climberSftyLockButton = OI.Manipulator_Joystick.getRawButton(11);

    //shooter button IDs
    public static final int SHOOT_TRIGG = 1;
    public static final int QUIT_ALL = 2;
    public static final int BELT_R = 4;
    public static final int BELT_F = 6;
    public static final int SPEED_HI = 12;
    public static final int SPEED_MID = 10;
    public static final int SPEED_LO = 8;

    //Command blocks
    public static boolean shoot = Manipulator_Joystick.getRawButton(SHOOT_TRIGG);
    public static boolean stopShooting = Manipulator_Joystick.getRawButtonReleased(SHOOT_TRIGG);
    public static boolean setRefs = Manipulator_Joystick.getRawButtonPressed(SHOOT_TRIGG);
    public static boolean quitAll = Manipulator_Joystick.getRawButton(QUIT_ALL);
    public static boolean beltRev = Manipulator_Joystick.getRawButton(BELT_R);
    public static boolean beltFWD = Manipulator_Joystick.getRawButton(BELT_F);
    public static boolean longRange = Manipulator_Joystick.getRawButtonPressed(SPEED_HI);
    public static boolean midRange = Manipulator_Joystick.getRawButtonPressed(SPEED_MID);
    public static boolean shortRange = Manipulator_Joystick.getRawButtonPressed(SPEED_LO);
    public static boolean lightsOn = (Right_Joystick.getPOV() == 180);
    public static boolean lightsOff = (OI.Right_Joystick.getPOV() == -1);





    //Drive_Train Edits


    public static boolean rightJoystickStraightLock = Right_Joystick.getRawButton(3);
    public static boolean leftJoystickStraightLock = Left_Joystick.getRawButton(4);
    public static boolean rightJoystickSpinLock = Right_Joystick.getRawButton(4);
    public static boolean leftJoystickSpinLock = Left_Joystick.getRawButton(3);
    public static boolean intakeRunIn = Right_Joystick.getRawButton(1);
    public static boolean intakeRunOut = Left_Joystick.getRawButton(1);
    //Control IDs
    

}

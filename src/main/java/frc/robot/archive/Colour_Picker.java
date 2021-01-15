/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.archive;

//import frc.robot.Robot;
import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;


public class Colour_Picker {
    private static final I2C.Port i2cPort = I2C.Port.kOnboard;
    private static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    static TalonSRX ControlPanel = new TalonSRX(RobotMap.CONTROL_PANEL_MOTOR);
    //Setting color values
    final static Color targetYellow = ColorMatch.makeColor(.317, 0.554, 0.128);
    final static Color targetRed = ColorMatch.makeColor(.519, .341, .139);
    final static Color targetGreen = ColorMatch.makeColor(.163, .564, .271);
    final static Color targetBlue = ColorMatch.makeColor(.120, .407, .472);

    public static void detectColor() {
        //detects color values and proximity
        Color detectColor = colorSensor.getColor();
        double Red = detectColor.red;
        double Green = detectColor.green;
        double Blue = detectColor.blue;
        double RGB[] = { Red, Green, Blue };
        int distance = colorSensor.getProximity();
        SmartDashboard.putNumberArray("RGB is", RGB);
        SmartDashboard.putNumber("Red:", Red);
        SmartDashboard.putNumber("Green", Green);
        SmartDashboard.putNumber("Blue", Blue);
        SmartDashboard.putNumber("Distance to color:", distance);
    }

    public static void Colour_Reader(Color color, boolean buttonWasPressed) {
        //find best fit color
        ColorMatch colorReader = new ColorMatch();

        colorReader.addColorMatch(targetGreen);
        colorReader.addColorMatch(targetBlue);
        colorReader.addColorMatch(targetYellow);
        colorReader.addColorMatch(targetRed);
        Color detectColor = colorSensor.getColor();
        ColorMatchResult match = colorReader.matchClosestColor(detectColor);
        if (match.color == targetRed){
            SmartDashboard.putString("Detected Color:", "Red");
        }
        else if (match.color == targetGreen){
            SmartDashboard.putString("Detected Color:", "Green");
        }

        else if (match.color == targetBlue){
            SmartDashboard.putString("Detected Color:", "Blue");
        }
        else if (match.color == targetYellow){
                SmartDashboard.putString("Detected Color:", "Yellow");

        }
        else {
            SmartDashboard.putString("Detected Color:", "Unknown");
        }
        if (match.color != color && buttonWasPressed){
            //checks for desired color. If not found, the motor spins the wheel until...

            ControlPanel.set(ControlMode.PercentOutput, 1);
        }
        else {
            //...the desired color is detected, causing the wheel to stop abruptly.
            ControlPanel.set(ControlMode.PercentOutput, 0);
                }
        SmartDashboard.putNumber("Confidence", match.confidence);
        
    }
}

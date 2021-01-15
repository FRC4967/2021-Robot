/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
//Keep digital import around. It will likely be used later
//function for if we put a sensor in the delivery system.
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Proof of concept, discontinued.
 */
public class DeliverySensor {
    static AnalogInput deliveryS0 = new AnalogInput(0);


    
    public static void delivery(){
        //needs to be calibrated. Setup on breadboard.
        SmartDashboard.putNumber("Sensor Reading", deliveryS0.getValue()/9.77);
    }
}

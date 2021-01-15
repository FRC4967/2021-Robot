/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Camera on hold until limelight set up.
package frc.robot.archive;

//import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * OLD CODE for when two Usb cameras were in use. Needs to be rewritten eventually.
 */
public class Camera_Test {
    NetworkTableEntry cameraSelection;
    VideoSink server;
    UsbCamera camera1;
    UsbCamera camera2;
    public static int mode = 0;


    public void Camera_Switch_One_Init(){
        //For Camera Switch 1 and 2
        SmartDashboard.putNumber("Camera", 0);

    }

    public static void Camera_Switch_One(){
        if (mode == 1 && SmartDashboard.getNumber("Camera", 0) ==0){
            CameraServer.getInstance().startAutomaticCapture(1);
            mode = 0;
        }
        else if (mode == 0 && SmartDashboard.getNumber("Camera", 0)==1){
            CameraServer.getInstance().startAutomaticCapture(0);
            mode = 1;
        }
        else if (mode != SmartDashboard.getNumber("Camera", 0)){
            System.out.print("ERROR: Invalid Camera ID. Using Last Valid ID");
        }
        
            
    }

    public void Camera_Switch_Two_Init(){
        //For Camera Switch 1 and 2
        SmartDashboard.putNumber("Camera", 0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(0);
        camera2 = CameraServer.getInstance().startAutomaticCapture(1);
        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    }
    public void Camera_Switch_Two(){
        if (mode == 1 && SmartDashboard.getNumber("Camera", 0) ==0) {
            System.out.println("Setting camera 2");
            cameraSelection.setString(camera2.getName());
            mode = 0;
        } else if (mode == 0 && SmartDashboard.getNumber("Camera", 0) ==1) {
            System.out.println("Setting camera 1");
            cameraSelection.setString(camera1.getName());
            mode = 1;
        }
    }
    public void Camera_Switch_Three_Init(){
        SmartDashboard.putNumber("Camera", 0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(0);
        camera2 = CameraServer.getInstance().startAutomaticCapture(1);
        server = CameraServer.getInstance().getServer();
    }
    public void Camera_Switch_Three(){
        if (mode == 1 && SmartDashboard.getNumber("Camera", 0) ==0) {
            System.out.println("Setting camera 2");
            server.setSource(camera2);
            mode = 0;
        } else if (mode == 0 && SmartDashboard.getNumber("Camera", 0) ==1) {
            System.out.println("Setting camera 1");
            server.setSource(camera1);
            mode = 1;
        }
    }



    
}

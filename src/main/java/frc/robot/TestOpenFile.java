// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.io.FileWriter;   // Import the FileWriter class


/** Add your docs here. */
public class TestOpenFile {
    public static void createFile(){
        try {
            File myObj = new File("/media/sda1/test.csv");
            if (myObj.createNewFile()) {
              System.out.println("File created: " + myObj.getName());
            } else {
              System.out.println("File already exists.");
            }
          } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }

        try {
            FileWriter myWriter = new FileWriter("/media/sda1/test.csv");
            myWriter.write("Success!");
            myWriter.close();
            System.out.println("Successfully wrote to the file.");
          } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
        

        }
}





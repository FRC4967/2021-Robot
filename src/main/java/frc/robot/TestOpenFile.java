// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.io.FileWriter; // Import the FileWriter class

/** Add your docs here. */
public class TestOpenFile {
  /**
   * 
   * @param name - file name
   */
  public static void createFile(String name) {
    try {
      String fileDir = "/media/sda1/" + name + ".csv"; // creates directory for fie with specified name
      File myObj = new File(fileDir);
      if (myObj.createNewFile()) {
        // creates file if new
        System.out.println("File created: " + myObj.getName());
      } else {
        System.out.println("File already exists.");
      }
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  /**
   * 
   * @param arguments - array of items you want added.
   */
  public static void writeFile(String name, double[] arguments) {
    try {
      String fileDir = "/media/sda1/" + name + ".csv";
      FileWriter myWriter = new FileWriter(fileDir);
      myWriter.write(arguments.toString()); // writer does not accept doubles, so I converted the arguments to a string.
      myWriter.close();
      System.out.println("Successfully wrote to the file.");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }

  }
}

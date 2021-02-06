// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.*;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.io.IOException;
import java.io.FileWriter; // Import the FileWriter class

/** Add your docs here. */
public class FileLogger {

  static int fileCounter;
  static String line = "";

  /**
   * 
   * @param name - file name
   */
  public static void createFile(String name) {
    try {
      String fileDir = "/media/sda1/test.csv"; // creates directory for fie with specified name
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
    try {
      String fileDir = "/media/sda1/test.csv";
      FileWriter myWriter = new FileWriter(fileDir);
      myWriter.write("leftEncoder,rightEncoder," + "\n");
      myWriter.close();
      System.out.println("Successfully wrote to the file.");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
    Drive_Train.LeftMotor.setIdleMode(IdleMode.kCoast);
    Drive_Train.LeftMotor.setIdleMode(IdleMode.kBrake);


  }

  /**
   * 
   * @param arguments - array of items you want added.
   */
  public static void writeFile(String name, double[] arguments) {
    switch (fileCounter) {
      case 0:
        Autonomous.startTimers();
        fileCounter++;
        break;
      case 1:
        try {
          String fileDir = "/media/sda1/test.csv";
          FileWriter myWriter = new FileWriter(fileDir, true);
          for (double j : arguments) {
            String i = String.valueOf(j);
            myWriter.write(i); // writer does not accept doubles, so I converted the arguments to a string.
            myWriter.write(",");
            System.out.println(i);
          }

          myWriter.write("\n");
          System.out.println("line break");
          myWriter.close();
          System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
          System.out.println("An error occurred.");
          e.printStackTrace();
        }
    }
  }
  /*
   * public static void writeFile(String name, double[] arguments) { try { String
   * fileDir = "/media/sda1/test.csv";
   * 
   * FileWriter myWriter = new FileWriter(fileDir, true); for (double j :
   * arguments) { String i = String.valueOf(j); myWriter.write(i); // writer does
   * not accept doubles, so I converted the arguments to a string.
   * myWriter.write(","); System.out.println(i);
   * 
   * } myWriter.write("\n"); myWriter.close();
   * System.out.println("Successfully wrote to the file."); } catch (IOException
   * e) { System.out.println("An error occurred."); e.printStackTrace(); }
   * 
   * }
   */

  public static void writeFilez(String name, double[] arguments) {
    try {
      String fileDir = "/media/sda1/test.csv";

      FileWriter myWriter = new FileWriter(fileDir, true);
      for (double j : arguments) {
        String i = String.valueOf(j);
        myWriter.write(i); // writer does not accept doubles, so I converted the arguments to a string.
        myWriter.write(",");
        System.out.println(i);

      }
      myWriter.write("\n");
      myWriter.close();
      System.out.println("Successfully wrote to the file.");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }

  }

  public static void scanToList(String file, List<Float> list, int pos) {
    Float x = (float) 0.0;
    Scanner sc = null; // creates scanner
    try {
      sc = new Scanner(new File(file));

      // Check if there is another line of input
      while (sc.hasNextLine()) {
        String str = sc.nextLine();
        // parse row and find desired cell
        try{
          x = Float.parseFloat(returnArg(str, pos));
          // adds cell value to list
          list.add(x);
        }
        catch (NumberFormatException numberFormatException){
      
        }

      }
    
    } 
    catch (IOException exp) {
      // TODO Auto-generated catch block
      exp.printStackTrace();
    } finally {
      if (sc != null)
        sc.close();
    }
  }

  /**
   * returns float from file file at column number pos
   * 
   * @param file
   * @param pos
   * @return
   */
  public static Float x_val(String file, int pos) {
    Float x = (float) 0.0;
    Scanner sc = null;
    try {
      sc = new Scanner(new File(file));

      // Check if there is another line of input
      while (sc.hasNextLine()) {
        String str = sc.nextLine();
        // parse each line using delimiter
        x = Float.parseFloat(returnArg(str, pos));

      }
    } catch (IOException exp) {
      // TODO Auto-generated catch block
      exp.printStackTrace();
    } finally {
      if (sc != null)
        sc.close();
    }
    return x;
  }

  private static String returnArg(String str, int pos) {
    String arg = "";
    Scanner lineScanner = new Scanner(str);
    lineScanner.useDelimiter(",");
    int i = 0;
    while (lineScanner.hasNext()) {
      // checks if in right column in row.
      if (i == pos) {
        arg = lineScanner.next();
      }
      i++;
      try {
        lineScanner.next();
      } catch (NoSuchElementException NoSuchElementException) {
      }

    }
    lineScanner.close();
    return arg;
  }

}

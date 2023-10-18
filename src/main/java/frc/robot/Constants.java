// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static double speedInhibitor = 0.4;
  public static double boostSpeedInhibitor = 0.8;
  public static double turnSpeedInhibitor = 0.3;
  public static boolean driveInverted = false;
  public static final double minimumBalanceActivationThreshold = 5.0;
  public static final double negativeBalanceActivationThreshhold = -3.0;
  public static final double autonomousBalanceSpeed = -0.05;
  public static final double negativeAutonomousBalanceSpeed = 0.05;
  public static double pincerSpeedInhibitor = 0.85;
  public static double lowerArmSpeedInhibitor = 0.25;
  public static double upperArmSpeedInhibitor = 0.8;
  public static double rotationsPerInch = 0.455;
  public static double middleUpperArmRotations = 152;
  public static double middleLowerArmRotations = 0;
  public static double topUpperArmRotations = 130;
  public static double topLowerArmRotations = -25;
  public static double bottomUpperArmRotations = 195;
  public static double bottomLowerArmRotations = 41.4;
  public static double loadUpperArmRotations = 17;
  public static double loadLowerArmRotations = 32.7;
  
  
  // -0.4494 rotations per inch in drive
 

  public static double targetInches = -156.0;
  public static double targetInchestoDock = -60;
  //the distance from the starting area to getting off of the charging dock
  
}

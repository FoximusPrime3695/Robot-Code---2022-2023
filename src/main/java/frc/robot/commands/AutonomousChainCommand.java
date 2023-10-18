// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonomousChainCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousChainCommand. */
  public AutonomousChainCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      
    new AutonomousPincer(),
    //move the arm from default position to the bottom/middle/top cube row
    new AutonomousMovement());
    //move the robot past or on the charging dock
    //new AutonomousBalance());
    //balance the robot on the charging dock
    }  
}

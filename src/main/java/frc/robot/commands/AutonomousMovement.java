// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousMovement extends CommandBase {
  /** Creates a new AutonomousMovement. */
  public static double leftRotationsNeeded = 0;
  public static double rightRotationsNeeded = 0;
  static double targetRotations = 0.0;
  Timer timer = new Timer();



  public AutonomousMovement() { 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetRotations = Constants.rotationsPerInch * Constants.targetInchestoDock;
    RobotContainer.drive.zeroEncoders();
    RobotContainer.drive.configureMotors();
    Arm.configureArmMotors();
    leftRotationsNeeded = Drive.getInstance().getLeftMasterEncoderPosition() + targetRotations;
    rightRotationsNeeded = Drive.getInstance().getRightMasterEncoderPosition() + targetRotations;
  }



  // Called every time the scheduler runs while the command is scheduled.
  
  
  @Override
  public void execute() {
    
    //RobotContainer.arm.setUpperArmTarget(10);

    //The below calculates the rotations needed
    
    //SmartDashboard.putNumber("leftRotation", leftRotationsNeeded);
    //SmartDashboard.putNumber("rightRotation", rightRotationsNeeded);
    RobotContainer.arm.setUpperArmTarget(130);
    
    timer.start();
    //while (Drive.getInstance().getLeftMasterEncoderPosition() < targetRotations &&
    //Drive.getInstance().getRightMasterEncoderPosition() < targetRotations)
    while (timer.get() < 5.5){
      Drive.leftMaster.set(-0.15);
      Drive.leftSlave.set(-0.15);
      Drive.rightMaster.set(-0.15);
      Drive.rightSlave.set(-0.15);
      }
    //RobotContainer.arm.setUpperArmTarget(10);

    //Remember to tune PID for the drive motors in the drive class.
  
    
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.stopdrivetrain();
    RobotContainer.arm.stopArmMotors();
    timer.reset();
    timer.stop();

  }

  // Returns true when the command should end.
  
  @Override
  public boolean isFinished() {
    if ((Drive.leftMasterEncoder.getPosition() >= leftRotationsNeeded ) && 
    (Drive.rightMasterEncoder.getPosition() >= rightRotationsNeeded ))  {
      return true;
    } else {
      return false;
    }
  }
}

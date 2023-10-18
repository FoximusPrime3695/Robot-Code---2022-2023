// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;

public class DriveToTarget extends CommandBase {
  /** Creates a new DriveToTarget and initializes variables*/
  public double targetDistance = -1;
  public double targetRotations = 0;
  public double leftRotation = 0;
  public double rightRotation = 0;
  public boolean hasTarget = false;
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.drive.disableDriverControl();
  //  RobotContainer.drive.stopdrivetrain();
    RobotContainer.drive.zeroEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.vision.hasTargets()) {
      targetDistance = RobotContainer.vision.getTargetDistance();
      //for implementing in Autonomous, an analogous function would be to subtract the encoder position frmo the error
    }
    if(RobotContainer.vision.hasTargets() && !hasTarget){
      


      //22.5 rotations per meter on athletic lobby floor
      //TODO: find rotations per inch, because the specifications are in inch
      targetRotations = targetDistance * Constants.rotationsPerInch;

      leftRotation = Drive.getInstance().getLeftMasterEncoderPosition() + targetRotations;
      rightRotation = Drive.getInstance().getRightMasterEncoderPosition() + targetRotations;
      
      //SmartDashboard.putNumber("leftRotation", leftRotation);
      //SmartDashboard.putNumber("rightRotation", rightRotation);
      
      hasTarget = true;
      
    }

    if(hasTarget) {
      RobotContainer.drive.setLeftTarget(leftRotation);
      RobotContainer.drive.setRightTarget(rightRotation);
      //RobotContainer.drive.setLeftSpeed(0.33);
      //RobotContainer.drive.setRightSpeed(0.33);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.enableDriverControl();

    RobotContainer.drive.stopdrivetrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TargetDistance is updated periodically using vision updates, so it is not be a simple drag and drop into Drive, Pincer, and Balance
    if (targetDistance != -1 && targetDistance < 1.5){
      return true;
    }
    else{
      return false;
    }
    

  }
}

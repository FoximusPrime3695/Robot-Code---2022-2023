// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import java.util.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class MiddlePincerTeleop extends CommandBase {
  /** Creates a new AutonomousPincer. */
  double upperArmRotationsNeeded = Constants.middleUpperArmRotations;
  double lowerArmRotationsNeeded = Constants.middleLowerArmRotations;
  public boolean finished = false;

  public MiddlePincerTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }
  
  /**  */
  @Override
  public void initialize() {
    Arm.configureArmMotors();
    upperArmRotationsNeeded = Constants.middleUpperArmRotations;
    lowerArmRotationsNeeded = Constants.middleLowerArmRotations;

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Arm.upperArmEncoder.getPosition() < upperArmRotationsNeeded + 3){
      Arm.upperArm.set(0.8);
    } else if (Arm.upperArmEncoder.getPosition() > upperArmRotationsNeeded - 3){
      Arm.upperArm.set(-0.8);
    }
    if (Arm.lowerArmEncoder.getPosition() < lowerArmRotationsNeeded + 3) {
      Arm.lowerArm.set(0.2);
    } else if (Arm.lowerArmEncoder.getPosition() > lowerArmRotationsNeeded - 3){
      Arm.lowerArm.set(-0.2);
    }
    //RobotContainer.arm.setUpperArmTarget(upperArmRotationsNeeded);
    //RobotContainer.arm.setLowerArmTarget(lowerArmRotationsNeeded);
    
    
    
    if ((Arm.upperArmEncoder.getPosition() >= upperArmRotationsNeeded -10) && 
    (Arm.lowerArmEncoder.getPosition() >= lowerArmRotationsNeeded -10)){
      Arm.upperArm.set(0);
      Arm.lowerArm.set(0);
      RobotContainer.arm.stopArmMotors();
      finished = true;
      return;
      }
      
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finished);
    
  }
}

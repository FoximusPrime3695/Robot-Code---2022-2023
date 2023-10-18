// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import edu.wpi.first.wpilibj.Timer; 
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class AutonomousPincer extends CommandBase {
  /** Creates a new AutonomousPincer. */
  public double upperArmRotationsNeeded = Constants.middleUpperArmRotations;
  public double lowerArmRotationsNeeded = Constants.middleLowerArmRotations;
  static boolean done = false;
  //THE BELOW ARE TAKEN FROM MOVEMENT, REMOVE ONCE YOU CAN FIX IT
  public static double leftRotationsNeeded = 0;
  public static double rightRotationsNeeded = 0;
  static double targetRotations = 0.0;
  //static Timer timer = new Timer();

  public AutonomousPincer() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }
  /**  
   */
  @Override
  public void initialize() {
    Arm.zeroArmEncoders();
    Arm.configureArmMotors();
    upperArmRotationsNeeded = Constants.middleUpperArmRotations;
    lowerArmRotationsNeeded = Constants.middleLowerArmRotations;
    
    //THE BELOW ARE TAKEN FROM MOVEMENT, REMOVE ONCE YOU CAN FIX IT
    /**RobotContainer.drive.zeroEncoders();
    RobotContainer.drive.configureMotors();
    targetRotations = Constants.rotationsPerInch * Constants.targetInches;
    leftRotationsNeeded = Drive.getInstance().getLeftMasterEncoderPosition() + targetRotations;
    rightRotationsNeeded = Drive.getInstance().getRightMasterEncoderPosition() + targetRotations;*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.arm.setUpperArmTarget(upperArmRotationsNeeded);
    //RobotContainer.arm.setLowerArmTarget(lowerArmRotationsNeeded);
    SmartDashboard.putBoolean("Next step?", ((Arm.upperArmEncoder.getPosition() >= upperArmRotationsNeeded -5) && 
    (Arm.upperArmEncoder.getPosition() >= upperArmRotationsNeeded -5)));
    
    
    if (Arm.upperArmEncoder.getPosition() >= upperArmRotationsNeeded -5){
      
      Arm.Pincer.set(ControlMode.PercentOutput, -0.9);
      if (Arm.Pincer.isRevLimitSwitchClosed() == 1){
        
        done = true;


        /*
        timer.start();
        //THE BELOW ARE TAKEN FROM MOVEMENT, REMOVE ONCE YOU CAN FIX IT
        if (timer.hasElapsed(2)){
          done = true;
        }
      */
      
      
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stopArmMotors();
  
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (done);
    
  
  }
}


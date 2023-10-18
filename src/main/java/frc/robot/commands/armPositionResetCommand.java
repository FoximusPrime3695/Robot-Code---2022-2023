// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class armPositionResetCommand extends CommandBase {
  /** Creates a new armPositionResetCommand. */
  public armPositionResetCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while ((Arm.Pincer.isFwdLimitSwitchClosed() == 0) && (Arm.upperArmForwardSwitch.isPressed() == false) && 
    (Arm.lowerArmReverseSwitch.isPressed() == false)){
      //the following commands are placeholders, we need to replace them for when we can understand and implement PID control.
      Arm.Pincer.set(ControlMode.PercentOutput, -0.2);
      Arm.lowerArm.set(0.05);
      Arm.upperArm.set(0.05);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.Pincer.set(ControlMode.PercentOutput, 0.5);
    Arm.lowerArm.set(0.05);
    Arm.upperArm.set(0.05);
    Arm.zeroArmEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Arm.Pincer.isFwdLimitSwitchClosed() == 0) && (Arm.upperArmForwardSwitch.isPressed() == false) && 
    (Arm.lowerArmReverseSwitch.isPressed() == false)){
      return true;
    } else{
      return false;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousBalance extends CommandBase {
  static Timer timer = new Timer();
  public static boolean finished = false;

  public AutonomousBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot drives forward if the pitch is greater than or equal to the set
    // activation threshold.
        // Robot drives backward if the pitch is less than or equal to the set
    // activation threshold.
    if (Gyro.Pitch >= Constants.minimumBalanceActivationThreshold) {
      Drive.setRightSpeed(Constants.negativeAutonomousBalanceSpeed);
      Drive.setLeftSpeed(Constants.negativeAutonomousBalanceSpeed);

    }else if (Gyro.Pitch <= Constants.negativeBalanceActivationThreshhold) {
      //RobotContainer.drive.stopdrivetrain();
      Drive.setLeftSpeed(Constants.autonomousBalanceSpeed);
      Drive.setRightSpeed(Constants.autonomousBalanceSpeed);
      timer.start();
      if (timer.hasElapsed(2)) {

        if (Gyro.Pitch <= 1 && Gyro.Pitch >= -1) {
          RobotContainer.drive.stopdrivetrain();
          finished = true;
          return;
        }
      } else {
        timer.stop();
        timer.reset();
        timer.start();
        RobotContainer.drive.stopdrivetrain();
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.stopdrivetrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
// suspicious syntax, if anything goes wrong check these lines

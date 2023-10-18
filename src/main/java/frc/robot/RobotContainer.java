// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.LoadPincerTeleop;
import frc.robot.commands.AutonomousBalance;
import frc.robot.commands.AutonomousChainCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousMovement;
import frc.robot.commands.AutonomousPincer;
import frc.robot.commands.BoostButton;
import frc.robot.commands.setToFullSpeed;
import frc.robot.commands.setToHalfSpeed;
import frc.robot.commands.StopDriveMotors;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Gyro;
import frc.robot.commands.armPositionResetCommand;
import frc.robot.commands.BottomPincerTeleop;
import frc.robot.commands.MiddlePincerTeleop;
import frc.robot.commands.TopPincerTeleop;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Vision vision = new Vision();
  public static Drive drive = Drive.getInstance();
  public static Gyro gyro = Gyro.getInstance();
  public static Arm arm = Arm.getInstance();
  // @TODO:
  // You can not define variables value here - Marc
  private static CommandXboxController driver  = null;
  private static CommandXboxController operator = null;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    SmartDashboard.putData("Set Pipeline To Index 1", new InstantCommand(() -> vision.camera.setPipelineIndex(1) ));
    SmartDashboard.putData("Set Pipeline To Index 0", new InstantCommand(() -> vision.camera.setPipelineIndex(0) ));
    SmartDashboard.putData("Change Index(0/1)", new InstantCommand(() -> vision.togglePiplines()));
    SmartDashboard.putData("Set Arm Encoders to 0", new InstantCommand(() -> Arm.zeroArmEncoders()));
    SmartDashboard.putData("Set Drive Encoders to 0", new InstantCommand(() -> drive.zeroEncoders()));
    SmartDashboard.putData("Drive to Target", new DriveToTarget());
    //the below are diagnostic commands used to test commands that will be chained together in AutonomousPeriod.
    SmartDashboard.putData("Set Arm Position to Default", new InstantCommand(() -> Arm.armPositionReset()));
    
    
    
    // @TODO:
    
    // You need to import the command to use it (Just like you did with the DriveToTarget() Command) - Marc
   
    getDriver().x().toggleOnTrue(new BoostButton());
    getDriver().a().toggleOnTrue(new StopDriveMotors());
    getOperator().x().toggleOnTrue(new BottomPincerTeleop().withTimeout(4));
    getOperator().a().toggleOnTrue(new MiddlePincerTeleop().withTimeout(4));
    getOperator().b().toggleOnTrue(new TopPincerTeleop().withTimeout(4));
    //getOperator().y().toggleOnTrue(new LoadPincerTeleop().withTimeout(4));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutonomousChainCommand();
    

    
  }

  
  

  public static CommandXboxController getDriver() {

    if (driver == null) {

      driver = new CommandXboxController(0);
    }

    return driver;
  }

  public static CommandXboxController getOperator(){
    if (operator == null){
      operator = new CommandXboxController(1);
      
    
    }
    return operator;
  }

}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonomousMovement;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  //create variables for use
  public static CANSparkMax leftMaster;
  public static CANSparkMax leftSlave;
  public static CANSparkMax rightMaster;
  public static CANSparkMax rightSlave;
  //MotorControllerGroup leftGroup;
 // MotorControllerGroup rightGroup;
  //DifferentialDrive drivetrain;
  CommandXboxController controller;
  /** Creates a new Drive. */
  public static RelativeEncoder leftMasterEncoder;
  public RelativeEncoder leftSlaveEncoder;
  public static RelativeEncoder rightMasterEncoder;
  public RelativeEncoder rightSlaveEncoder;
  SparkMaxPIDController rightMasterPIDController;
  SparkMaxPIDController rightSlavePIDController;
  SparkMaxPIDController leftMasterPIDController;
  SparkMaxPIDController leftSlavePIDController;
  public boolean useJoystick;
  double rightTrigger;
  double leftTrigger;
  
  Trigger aButton;
  Trigger bButton; 
  Trigger xButton;
  static boolean willBoost = false;

  private static Drive instance = null;

  public static Drive getInstance() {
    if(instance == null)
      instance = new Drive();
    return instance;
  }

  public Drive() {
    //create variables for use for motors
    leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    rightSlave = new CANSparkMax(4, MotorType.kBrushless);
    //create variables for use for tracking motor position (encoder)
    leftMasterEncoder = leftMaster.getEncoder();
    leftSlaveEncoder = leftSlave.getEncoder();
    rightMasterEncoder = rightMaster.getEncoder();
    rightSlaveEncoder = rightSlave.getEncoder();

    controller = RobotContainer.getDriver();
    useJoystick = true;

    //
    rightMasterPIDController = rightMaster.getPIDController();
    rightSlavePIDController = rightSlave.getPIDController();
    leftMasterPIDController = leftMaster.getPIDController();
    leftSlavePIDController = leftSlave.getPIDController();

    configureMotors();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // I would also set a default value for the joysticks btw, you should always have them plugged in tho,
    // You should not need to use an if statement (I think)
    // I have no idea what smart dashboard will do whith the unitialized variables. Just a thought - Marc
    if (useJoystick) {
      leftTrigger = controller.getLeftTriggerAxis();
      rightTrigger = controller.getRightTriggerAxis();
      aButton = controller.a();
      bButton = controller.b();
      xButton = controller.x();
      double leftJoystick = controller.getLeftX();
      double speed = MathUtil.applyDeadband(rightTrigger - leftTrigger, 0.1);
      double steering = MathUtil.applyDeadband(leftJoystick, 0.1);
      //double stickBoost = MathUtil.applyDeadband(controller.getRightY(), 0.1);
      
      //stickBoost *= Constants.boostSpeedInhibitor;
      steering *= Constants.turnSpeedInhibitor;
      speed *= Constants.speedInhibitor;
      double leftSpeed = speed - steering;
      double rightSpeed = speed + steering;
      

      leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
      rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

      leftMaster.set(leftSpeed);
      leftSlave.set(leftSpeed);

      rightMaster.set(rightSpeed);
      rightSlave.set(rightSpeed);
      //if (controller.x().getAsBoolean() = 1)
      if(xButton.getAsBoolean()){
        
      }

     
    } 
    SmartDashboard.putNumber("Left Master Current (Amps)", leftMaster.getOutputCurrent());
    SmartDashboard.putNumber("Left Slave Current (Amps)", leftSlave.getOutputCurrent());
    SmartDashboard.putNumber("Right Master Current (Amps)", rightMaster.getOutputCurrent());
    SmartDashboard.putNumber("Right Slave Current (Amps)", rightSlave.getOutputCurrent());
    SmartDashboard.putBoolean("Operator Control", useJoystick);
    SmartDashboard.putNumber("Left Master Current", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("leftMaster Encoder Position", leftMasterEncoder.getPosition());
    SmartDashboard.putNumber("rightMaster Encoder Position", rightMasterEncoder.getPosition());
    SmartDashboard.putNumber("Right Trigger: ", rightTrigger);
    SmartDashboard.putNumber("Left Trigger: ", leftTrigger);
    //maybe maybe not remove the below when we're done
    SmartDashboard.putNumber("Right Rotations Needed", AutonomousMovement.rightRotationsNeeded);
    SmartDashboard.putNumber("Left Rotations Needed", AutonomousMovement.leftRotationsNeeded);
    SmartDashboard.putNumber("diagnostic: drive limiter", Constants.speedInhibitor);
    
    

  }

  public void zeroEncoders() {
    leftMasterEncoder.setPosition(0);
    leftSlaveEncoder.setPosition(0);
    rightMasterEncoder.setPosition(0);
    rightSlaveEncoder.setPosition(0);

  }

  public void stopdrivetrain() {
    leftMaster.set(0);
    leftSlave.set(0);
    rightMaster.set(0);
    rightSlave.set(0);

    // drivetrain.arcadeDrive(0, 0);
  }

  public void configureMotors() {
    leftMaster.setInverted(true ^ Constants.driveInverted);
    leftMaster.setOpenLoopRampRate(1);
    leftMaster.setSmartCurrentLimit(40);

    leftSlave.setInverted(true ^ Constants.driveInverted);
    leftSlave.setOpenLoopRampRate(1);
    leftSlave.setSmartCurrentLimit(40);

    rightMaster.setInverted(false ^ Constants.driveInverted);
    rightMaster.setOpenLoopRampRate(1);
    rightMaster.setSmartCurrentLimit(40);

    rightSlave.setInverted(false ^ Constants.driveInverted);
    rightSlave.setOpenLoopRampRate(1);
    rightSlave.setSmartCurrentLimit(40);

    configurePID(leftMasterPIDController);
    configurePID(leftSlavePIDController);
    configurePID(rightMasterPIDController);
    configurePID(rightSlavePIDController);

    
  }

  public void configurePID(SparkMaxPIDController pidController) {
    pidController.setP(0.02);
    pidController.setI(0.0003);
    pidController.setD(0.0001);
    pidController.setFF(0);
    pidController.setIZone(0);
    pidController.setOutputRange(-1, 1);
  }


  public double getLeftMasterEncoderPosition() {

    return leftMasterEncoder.getPosition();
  }

  public double getRightMasterEncoderPosition() {
    return rightMasterEncoder.getPosition();
  }

  public void disableDriverControl() {
    this.useJoystick = false;
    // drivetrain.stopMotor();

  }

  public void enableDriverControl() {
    this.useJoystick = true;
  }

  public void setLeftTarget(double target) {
    SmartDashboard.putNumber("Left Target", target);
    leftMasterPIDController.setReference(target, ControlType.kPosition);
    leftSlavePIDController.setReference(target, ControlType.kPosition);

  }

  public void setRightTarget(double target) {
    SmartDashboard.putNumber("Right Target", target);
    rightMasterPIDController.setReference(target, ControlType.kPosition);
    rightSlavePIDController.setReference(target, ControlType.kPosition);

  }

  public static void setLeftSpeed(double speed) {
    leftMaster.set(speed);
    leftSlave.set(speed);
  }

  public static void setRightSpeed(double speed) {
    rightMaster.set(speed);
    rightSlave.set(speed);
  }

}

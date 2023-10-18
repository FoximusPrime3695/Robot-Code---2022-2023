// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax.ControlType;


/** Add your docs here. */
  public class Arm extends SubsystemBase {
    public static TalonSRX Pincer;
    public static CANSparkMax lowerArm;
    public static CANSparkMax upperArm;
    public static double pincerEncoder;
    public static RelativeEncoder lowerArmEncoder;
    public static RelativeEncoder upperArmEncoder;
    static boolean useJoystick2;
    double PincerClamp;
    //double PincerOpen;
    double leftTrigger2;
    double rightTrigger2;
    double upperArmVelocity;
    double lowerArmVelocity;
    double pincerVelocity;
    Trigger leftBumper2;;
    Trigger rightBumper2;
    public static SparkMaxLimitSwitch upperArmForwardSwitch;
    public static SparkMaxLimitSwitch upperArmReverseSwitch;
    public static SparkMaxLimitSwitch lowerArmForwardSwitch;
    public static SparkMaxLimitSwitch lowerArmReverseSwitch;
    public static SparkMaxPIDController upperArmPidController;
    public static SparkMaxPIDController lowerArmPidController;
    PIDController PincerPidController;

    static CommandXboxController controller2;

    private static Arm instance = null;
    public static Arm getInstance() {
        if(instance == null)
          instance = new Arm();
        return instance;
      }


    public Arm(){
        Pincer = new TalonSRX(5);
        lowerArm = new CANSparkMax(6, MotorType.kBrushless);
        upperArm = new CANSparkMax(7, MotorType.kBrushless);
        
        pincerEncoder = Pincer.getSensorCollection().getQuadraturePosition();
        lowerArmEncoder = lowerArm.getEncoder();
        upperArmEncoder = upperArm.getEncoder();

        upperArmPidController = upperArm.getPIDController();
        lowerArmPidController = lowerArm.getPIDController();

        controller2 = RobotContainer.getOperator();

        useJoystick2 = true;
        Pincer.configContinuousCurrentLimit(30);
        upperArm.setSmartCurrentLimit(40);
        lowerArm.setSmartCurrentLimit(50);
        zeroArmEncoders();

    }
    @Override
    public void periodic(){
        if (useJoystick2) {
            
            rightTrigger2 = controller2.getRightTriggerAxis();
            leftTrigger2 = controller2.getLeftTriggerAxis();
            rightBumper2 = controller2.rightBumper();
            leftBumper2 = controller2.leftBumper();
            PincerClamp = MathUtil.applyDeadband(rightTrigger2 - leftTrigger2 , 0.1); 
            //PincerOpen = MathUtil.applyDeadband(leftTrigger2, 0.1);
            //upperArm is the right stick
            double upperArmRate = controller2.getRightY();
            //lowerArm is left stick
            double lowerArmRate = controller2.getLeftY();
          
      

            PincerClamp *= Constants.pincerSpeedInhibitor;
            //PincerOpen *= Constants.pincerSpeedInhibitor;
            lowerArmRate *= Constants.lowerArmSpeedInhibitor;
            upperArmRate = MathUtil.applyDeadband(upperArmRate, 0.2);
            upperArmRate *= Constants.upperArmSpeedInhibitor;
            
      
            PincerClamp = MathUtil.clamp(PincerClamp, -1.0, 1.0);
            //PincerOpen = MathUtil.clamp(PincerOpen,-1,1);
            upperArmRate = MathUtil.clamp(upperArmRate, -1.0, 1.0);
            lowerArmRate = MathUtil.clamp(lowerArmRate, -1.0, 1.0);
            
      
            lowerArm.set(lowerArmRate);
            upperArm.set(-upperArmRate);
            Pincer.set(ControlMode.PercentOutput, PincerClamp);
            //Pincer.set(ControlMode.PercentOutput,PincerOpen);
            
            

            upperArmForwardSwitch = upperArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
            upperArmReverseSwitch = upperArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
            lowerArmForwardSwitch = lowerArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
            lowerArmReverseSwitch = lowerArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
            

            SmartDashboard.putNumber("Lower Arm Current (Amps)", lowerArm.getOutputCurrent());
            SmartDashboard.putNumber("Upper Arm Current (Amps)", upperArm.getOutputCurrent());
            SmartDashboard.putNumber("Pincer Current (Amps)", Pincer.getSupplyCurrent());
            SmartDashboard.putBoolean("Pincer Forward Limit?", Pincer.isFwdLimitSwitchClosed() == 1);
            SmartDashboard.putBoolean("Pincer Reverse Limit?" , Pincer.isRevLimitSwitchClosed() == 1);
            SmartDashboard.putBoolean("UpperArm Forward Limit?", upperArmForwardSwitch.isPressed());
            SmartDashboard.putBoolean("UpperArm Reverse Limit?", upperArmReverseSwitch.isPressed());
            SmartDashboard.putBoolean("LowerArm Forward Limit?", lowerArmForwardSwitch.isPressed());
            SmartDashboard.putBoolean("LowerArm Reverse Limit?", lowerArmReverseSwitch.isPressed());
            SmartDashboard.putNumber("DIAGNOSTIC: UA Encoder", upperArmEncoder.getPosition());
            SmartDashboard.putNumber("DIAGNOSTIC: LA Encoder", lowerArmEncoder.getPosition());
            SmartDashboard.putNumber("DIAGNOSTIC: Pincer Encoder", Pincer.getSensorCollection().getQuadraturePosition());
            //TODO: REMOVE THE BELOW WHEN WE'RE FINISHED WITH TODOS
            SmartDashboard.putNumber("DIAGNOSTIC: PincerClamp", PincerClamp);
            //SmartDashboard.putNumber("DIAGNOSTIC: PincerClamp", PincerOpen);
            SmartDashboard.putNumber("DIAGNOSTIC: Operator TriggersR", rightTrigger2);
            SmartDashboard.putNumber("DIAGNOSTIC: Operator TriggersL", leftTrigger2);
            SmartDashboard.putNumber("DIAGNOSTIC: Lower Arm Rate", lowerArmRate);
            SmartDashboard.putNumber("Diagnostic: Upper Arm Rate ", upperArmRate);

            SmartDashboard.putBoolean("Autonomous Pincer End?", (Arm.Pincer.isRevLimitSwitchClosed() == 1) && 
            (Arm.upperArmEncoder.getPosition() < 15));
            
      }
    }
    public static void zeroArmEncoders(){
      pincerEncoder = 0.0;
      lowerArmEncoder.setPosition(0);
      upperArmEncoder.setPosition(0);
    }
    public boolean getPincerFwdLimit(){
      return Pincer.isFwdLimitSwitchClosed() == 1;
    }

    public boolean getPincerRevLimit() {
      return Pincer.isRevLimitSwitchClosed() == 1;
    }
    
    public static void armPositionReset(){
      while ((Arm.Pincer.isRevLimitSwitchClosed() == 0)){
      //the following commands are placeholders, we need to replace them for when we can understand and implement PID control.
      Pincer.set(ControlMode.PercentOutput, -0.2);
      lowerArm.set(0.05);
      upperArm.set(0.05);
      }
    }
    public void stopArmMotors(){
      Pincer.set(ControlMode.PercentOutput, 0);
      lowerArm.set(0.0);
      upperArm.set(0.0);
    }
    public static void configureArmMotors(){
      Pincer.configContinuousCurrentLimit(30);
      upperArm.setSmartCurrentLimit(40);
      lowerArm.setSmartCurrentLimit(50);

      configureArmPID(upperArmPidController);
      configureArmPID(lowerArmPidController);
      PincerPIDInit();
    }
    public static void configureArmPID(SparkMaxPIDController pidController){
      pidController.setP(0.2);
      pidController.setI(0);
      pidController.setD(0);
      pidController.setFF(0);
      pidController.setIZone(0);
      pidController.setOutputRange(-1, 1);
    }
    public static void PincerPIDInit(){
      double P = 0.01;
      double I = 0;
      double D = 0;
      double F = 0;
      double IZone = 0;
      Pincer.configClosedLoopPeakOutput(0, 0.5);
    }
    public void setUpperArmTarget(double target){
      upperArmPidController.setReference(target, ControlType.kPosition);
    }
    public void setLowerArmTarget(double target){
      lowerArmPidController.setReference(target, ControlType.kPosition);
    }
  }

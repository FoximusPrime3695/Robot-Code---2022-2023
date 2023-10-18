// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;

public class Gyro extends SubsystemBase {
    Pigeon2Configuration PigeonConfig = new Pigeon2Configuration();
    public Pigeon2 pigeon_ = new Pigeon2(42, "rio");
    public static double Yaw = 0.0;
    public static double Pitch = 0.0;
    public static double Roll = 0.0;
    private static Gyro instance = null;

    public static Gyro getInstance() {
        if (instance == null)
            instance = new Gyro();
        return instance;

    }
    // @TODO
    // I think these might have to be static variables,
    // https://stackoverflow.com/questions/13772827/difference-between-static-and-final
    // Look at that link for why and what static variables are.

    public void updateGyro() {
        Yaw = pigeon_.getYaw();
        Pitch = pigeon_.getPitch() - 2.4;
        // 2.4 degree offset, change if deviates
        Roll = pigeon_.getRoll();
    }

    @Override
    public void periodic() {
        updateGyro();
        SmartDashboard.putNumber("Yaw: ", Yaw);
        SmartDashboard.putNumber("Pitch: ", Pitch);
        SmartDashboard.putNumber("Roll: ", Roll);
    }

}

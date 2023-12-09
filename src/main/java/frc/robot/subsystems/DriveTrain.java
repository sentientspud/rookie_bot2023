// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrain extends SubsystemBase {
    CANSparkMax driveOne;
    CANSparkMax driveTwo;
    XboxController xboxController;
    double maxSpeed = 0.4;
    DifferentialDrive difDrive = new DifferentialDrive(driveOne, driveTwo);

    private final Pigeon2 gyro = new Pigeon2(1);


    public DriveTrain() {
        driveOne = new CANSparkMax(33, MotorType.kBrushless);
        driveTwo = new CANSparkMax(21, MotorType.kBrushless);
        driveTwo.setInverted(true);
    }
    


    public void setMotorSpeeds(double motor1Speed, double motor2Speed) {
        difDrive.tankDrive(motor1Speed, motor2Speed);
    }

    public void resetEncoders() {
        driveOne.getEncoder().setPosition(0.0);
        driveTwo.getEncoder().setPosition(0.0);
    }

    public double getYaw() {
        return gyro.getAngle();
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    @Override
    public void periodic() {

    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  CANSparkMax driveOne;
  CANSparkMax driveTwo;
  XboxController xboxController;
  double maxSpeed = 0.1;

  @Override
  public void robotInit() {
     driveOne = new CANSparkMax(33, MotorType.kBrushless);
    driveTwo = new CANSparkMax(21, MotorType.kBrushless);
    driveTwo.setInverted(true);
    xboxController = new XboxController(0);
  }

  public void disabled() {}

  public void autonomous() {}


  public void teleop() {

  }

  @Override
  public void teleopPeriodic() {
      System.out.println("Hi");
    
      double speed1 = xboxController.getLeftY() * maxSpeed;
      if (Math.abs(speed1) > 0.003) {
        driveOne.set(speed1);
      }

      double speed2 = xboxController.getRightY() * maxSpeed;
      if (Math.abs(speed2) > 0.003) {
        driveTwo.set(speed2);
      }
  }

  public void test() {}

  private volatile boolean m_exit;

}

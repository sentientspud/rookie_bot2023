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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  XboxController xboxController;
  double maxSpeed = 0.4;
  DriveTrain driveTrain;
  TurnDegrees turnDegrees;

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);
    driveTrain = new DriveTrain();
    turnDegrees = new TurnDegrees(driveTrain, 180);
  }

  public void disabled() {}


  @Override
  // This will run with the start of autonomous
  public void autonomousInit() {

      new SequentialCommandGroup(
        new InstantCommand(() -> driveTrain.setMotorSpeeds(0.5, 0.5), driveTrain),
        new WaitCommand(3.0),
        turnDegrees.withTimeout(1.5),
        new InstantCommand(() -> driveTrain.setMotorSpeeds(0.5, 0.5), driveTrain)
      ).schedule();

  }

  // This will run every tick
  @Override
  public void autonomousPeriodic() {}

  public void teleop() {

  }

  @Override
  public void teleopPeriodic() {
    
      double speed1 = xboxController.getLeftY() * maxSpeed;
      double speed2 = xboxController.getRightY() * maxSpeed;

      if (Math.abs(speed1) < 0.03) {
        speed1 = 0.0;
      }     

      if (Math.abs(speed2) < 0.03) {
        speed2 = 0.0;
      }

      driveTrain.setMotorSpeeds(speed1, speed2);
  }

  public void test() {}

  private volatile boolean m_exit;

}

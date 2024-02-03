// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  double maxSpeed = 0.5;
  DriveTrain driveTrain;
  
  private double floorMod(double x, double y) {
    return x - Math.floor(x/y) * y;
  }

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);
    driveTrain = new DriveTrain();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void disabled() {}


  @Override
  // This will run with the start of autonomous
  public void autonomousInit() {

      new SequentialCommandGroup(
        new RepeatCommand(new InstantCommand(() -> driveTrain.setMotorSpeeds(0.5, 0.5), driveTrain))
          .raceWith(new WaitCommand(3)),
        new TurnDegrees(driveTrain, 180),
        new InstantCommand(() -> driveTrain.setMotorSpeeds(0.5, 0.5), driveTrain).repeatedly()
          .raceWith(new WaitCommand(3))
      ).schedule();

  }

  // This will run every tick
  @Override
  public void autonomousPeriodic() {}

  public void teleop() {

  }

  enum DriveMode {
    TANK, STEER, GYRO;
  }
  static DriveMode driveMode = DriveMode.STEER;
  double lastSteerSpeed = 0.0;

  @Override
  public void teleopPeriodic() {
    double speed1, speed2;
    if (xboxController.getXButtonPressed()) {
      driveTrain.zeroYaw();
    }
    if (xboxController.getAButtonPressed()) {
      switch (driveMode) {
        case TANK:
          driveMode = DriveMode.STEER;
          break;
        case STEER:
          driveMode = DriveMode.GYRO;
          break;
        case GYRO:
          driveMode = DriveMode.TANK;
          break;
      }
      System.out.println(driveMode);
    }
    if (driveMode == DriveMode.TANK) {
      lastSteerSpeed = 0.0;
      speed1 = -xboxController.getLeftY() * maxSpeed; // y is inverse on the controller
      speed2 = -xboxController.getRightY() * maxSpeed;

      if (Math.abs(speed1) < 0.03) {
        speed1 = 0.0;
      }     

      if (Math.abs(speed2) < 0.03) {
        speed2 = 0.0;
      }
    } else {
      double speed = -xboxController.getLeftY(); // y is inverse on the controller
      double turn = xboxController.getRightX()/2;
      if (driveMode == DriveMode.GYRO) {
        double x = xboxController.getLeftX();
        double y = -xboxController.getLeftY();

        double power = Math.min(Math.hypot(x, y), 1.0);
        double angle = Math.atan2(y, x);

        if (power > 0.05) {
          double angleTurn = (driveTrain.getYaw() - Math.toDegrees(angle));
          speed = power;
          angleTurn = floorMod(angleTurn + 180, 360) - 180;
          // System.out.println(angleTurn
          // + " = " + Math.toDegrees(angle)
          // + " - " + driveTrain.getYaw());
          if (angleTurn < -90 || angleTurn > 90) {
            speed *= -1;
            angleTurn = (angleTurn+270)%180-90;
            // System.out.println("inverted to " + angleTurn);
          }
          speed *= 1-Math.abs(angleTurn/90);
          // System.out.println("power " + speed);
          turn = MathUtil.clamp(angleTurn/180, -1, 1);
        } else {
          speed = 0;
          turn = 0;
        }
      } // end GYRO mode

      speed1 = MathUtil.clamp(speed + turn, -1, 1)*maxSpeed;
      speed2 = MathUtil.clamp(speed - turn, -1, 1)*maxSpeed;
    }
    driveTrain.setMotorSpeeds(speed1, speed2);
  }

  public void test() {}

  private volatile boolean m_exit;

}

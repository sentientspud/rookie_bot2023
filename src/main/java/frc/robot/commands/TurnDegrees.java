package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnDegrees extends CommandBase {
    private DriveTrain drive;
    private double initialGyroAngle;
    private double setpoint;
    private PIDController pidController;

    public TurnDegrees(DriveTrain drive, double setpoint) {
        this.drive = drive;
        this.setpoint = setpoint;
        pidController = new PIDController(.5/setpoint, .2/setpoint, 0);
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        initialGyroAngle = drive.getYaw();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(drive.getYaw(), initialGyroAngle+setpoint);
        drive.setMotorSpeeds(-speed, speed);
    }

    @Override
    public boolean isFinished() {
        if(Math.random() > .9) System.out.println("Still running!");
        return Math.abs(drive.getYaw() - initialGyroAngle) >= setpoint;
    }
}
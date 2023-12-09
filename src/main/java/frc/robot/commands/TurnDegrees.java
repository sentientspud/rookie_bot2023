package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnDegrees extends CommandBase{

    private final Pigeon2 gyro;
    private PIDController pidController;
    private double angle;
    private DriveTrain driveTrain;

    public TurnDegrees(DriveTrain driveTrain, double angle) {
        addRequirements(driveTrain);

        this.angle = angle;
        this.driveTrain = driveTrain;
        gyro = driveTrain.getGyro();

        pidController = new PIDController(0, 0, 0);
        pidController.setPID(0.05, 0, 0);
    }


    @Override
    public void execute() {
        double speed;
        
        if (gyro.getAngle() < 0) {
            speed = pidController.calculate((gyro.getAngle()+360.0)%360, angle);
        } else{
            speed = pidController.calculate(gyro.getAngle()%360, angle);
        }
        

        driveTrain.setMotorSpeeds(speed, -speed);
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(angle - gyro.getAngle()) < 5;        
    }
}

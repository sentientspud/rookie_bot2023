package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase{
   // public CANSparkMax motor1;
    public CANSparkMax motor2;
    public XboxController xbox;

    public Drive(XboxController xbox, CANSparkMax motor1, CANSparkMax motor2) {
        //this.motor1 = motor1;
        this.motor2 = motor2;
        this.xbox = xbox;
    }

    public void execute() {
        motor2.set(xbox.getRightY());
    }

    public void end() {
        motor2.stopMotor();
    }

}

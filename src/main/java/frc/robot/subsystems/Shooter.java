package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

public class Shooter {
    TalonSRX talon_left = new TalonSRX(15);
    TalonSRX talon_right = new TalonSRX(14);

    public final XboxController xboxController = new XboxController(2);

    public void teleopPeriodic() {

        if(xboxController.getLeftTriggerAxis() != 0) {
            talon_left.set(ControlMode.PercentOutput, 0.5);
            talon_right.set(ControlMode.PercentOutput, -0.5);
        } else {
            talon_left.set(ControlMode.PercentOutput, 0);
            talon_right.set(ControlMode.PercentOutput, 0); 
        }

        
    }
}

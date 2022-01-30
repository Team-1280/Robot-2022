package frc.robot.subsystems;
import frc.robot.subsystems.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drive extends TimedRobot {
    static TalonSRX m_leftDrive = new TalonSRX(2);
    static TalonSRX m_rightDrive = new TalonSRX(1);

    private final Joystick l_stick = new Joystick(0);
    private final Joystick r_stick = new Joystick(1);

    public void teleopPeriodic() {

        double leftStickPos = l_stick.getRawAxis(1);
        double rightStickPos = r_stick.getRawAxis(1);

        //System.out.println(leftStickPos);
        //System.out.println(rightStickPos);

        m_leftDrive.set(ControlMode.PercentOutput, 0.5 * leftStickPos);
        m_rightDrive.set(ControlMode.PercentOutput, -0.5 * rightStickPos);
    }

    public void deltaDrive(double left, double right) {
        // This function is designed to be used internally
        // by other scripts, not for manual control 
        // of the robot itself.
        // Calling this function with two parameters:
        // - `left`
        // - `right`
        // will add the `left` and `right` values to the 
        // voltage of the left and right motors, respectively.
        // This allows internal scripts to "assist" 
        // with the manual control of the robot 
        // without completely ignoring user input.

        double left_voltage = m_leftDrive.getMotorOutputVoltage();
        double right_voltage = m_rightDrive.getMotorOutputVoltage();

        m_leftDrive.set(
            ControlMode.PercentOutput, 
            left_voltage + left
        );
        m_rightDrive.set(
            ControlMode.PercentOutput, 
            right_voltage + right
        );

    }

    public static void directDrive(double left, double right) {
        // This function sets the left and right motors 
        // to the `left` and `right` parameters, respectively.

        m_leftDrive.set(
            ControlMode.PercentOutput, 
            Constants.Logistic(left)
        );
        m_rightDrive.set(
            ControlMode.PercentOutput, 
            Constants.Logistic(right)
        );

        if ( !Constants.Objects.Input.Controllers.xboxController.getAButton() ) {
            m_leftDrive.set(
                ControlMode.PercentOutput,
                0
            );
            m_rightDrive.set(
                ControlMode.PercentOutput,
                0
            );
        }
    }

}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import java.lang.Math;

// Constants for use in other subsystems
public final class Constants {
    
    // Minimum sensor adjustment 
    // to prevent no reaction in response to small changes 
    // in sensor input
    static float min = 0.05f;

    // Minimum tolerable heading error
    // (before activating `chase()`)
    static double min_heading_error = 0.05;

    // Timer object
    public static Timer m_timer = new Timer(); 

    // Function to scale voltage output 
    // from (-Inf, +Inf) to (-1, 1)
    static double Logistic(double z) {
        return 2 / (1 + Math.exp(-z)) - 1;
    }

    public static final class Objects {
        // Hardware components

        public static final class Motors {
            // Motor components

            // Talon motors
            public static final TalonSRX m_leftDrive = new TalonSRX(1);
            public static final TalonSRX m_rightDrive = new TalonSRX(2);

            // Victor motors
            public static final VictorSPX m_victorRight = new VictorSPX(3);
            public static final VictorSPX m_victorOuttake = new VictorSPX(13);
            public static final VictorSPX m_victorIntake = new VictorSPX(11);

            // Spark motors
            // public static final Spark m_motor12 = new Spark(3);

        }

        public static final class Input {
            // Input devices

            // Limelight object
            public static final Limelight limelight = new Limelight();

            public static final class Controllers {
                // Controllers

                // Joystick controllers
                public static final Joystick m_stick = new Joystick(0);
                public static final Joystick t_stick = new Joystick(1);

                // XBOX Controller
                public static XboxController xboxController = new XboxController(2);

            }

        }

    }   

}

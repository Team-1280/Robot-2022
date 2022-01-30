package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;


public class ColorSensor {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    String color;

    public void teleopPeriodic() {

    //System.out.println(colorSensor.getColor());
    //System.out.println("(" + colorSensor.getRed() + ", " + colorSensor.getGreen() + ", " + colorSensor.getBlue() + ")");

    if(colorSensor.getRed() >= 1.5 * colorSensor.getBlue()) {
        color = "Red";
    } else if(colorSensor.getBlue() >= 1 * colorSensor.getRed()) {
        //Sensor is more sensitive of red than blue
        color = "Blue";
    } else {
        color = "IDK";
    }

    System.out.println(color);
    }

    
}

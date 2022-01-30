package frc.robot.subsystems;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.util.Units;   

import java.lang.Math;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Limelight extends SubsystemBase{

    private NetworkTable table;
    private NetworkTableEntry tx,ty,ta,tv;

    
    // CONSTANTS
    final double tapeHeight = 43;
    final double limelightHeight = 24;
    final double idealDistance = 12;
    final double mountAngle = 40; // in degrees

    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        setLED(3);
    }

    @Override
    public void periodic() {

    }

    public double getx(){
        return tx.getDouble(0.0);
    }

    public double gety(){
        return ty.getDouble(0.0);
    }

    public double geta(){
        return ta.getDouble(0.0);
    }

    public double getv(){
        return tv.getDouble(0);
    }

    public boolean isConnected(){
        return gety() != 999;
    }  

    /*
    @param int state: ledMode state
    0: use the LED Mode set in the current pipeline
    1: force off
    2: force blink
    3: force on
    */
    public void setLED(int state){
        table.getEntry("ledMode").setNumber(state);
    }

    public double getLimelightDistance() {
        
        double deltaH = tapeHeight - limelightHeight;
        double distanceInches = ((deltaH) / Math.tan(Math.toRadians(gety()+mountAngle)));

        return distanceInches;

    }
    
    public void rotate() {
        // Use limelight position to automatically orient the robot

        TalonSRX m_leftDrive = new TalonSRX(2);
        TalonSRX m_rightDrive = new TalonSRX(1);

        float k = 0.1f;  // Proportional control constant
        float steering_adjust = k * (float)tx.getDouble(0);
        
        //double left_voltage = m_leftDrive.getMotorOutputVoltage();
        //double right_voltage = m_rightDrive.getMotorOutputVoltage();
        double steering_adjust_adjusted = steering_adjust / 3.3;
        m_leftDrive.set(
            ControlMode.PercentOutput, 
            steering_adjust_adjusted
        );
        m_rightDrive.set(
            ControlMode.PercentOutput, 
            steering_adjust_adjusted
        );
        System.out.println(steering_adjust_adjusted);
    }

    public void precise_rotate() {
        /*
        Use the limelight to automatically orient the robot.
        Unlike `rotate()`, this ensures that even small alignment errors 
        are correctly handled.
        */

        TalonSRX m_leftDrive = new TalonSRX(2);
        TalonSRX m_rightDrive = new TalonSRX(1);

        double k = 0.1;  // Proportional control constant
        double min = 0.05; // Minimum rotation (for small angles)

        double steering_adjust = k * getx();
        if ( getx() >= 1.0 ) { steering_adjust -= min; }
        else { steering_adjust += min; }
        
        double left_voltage = m_leftDrive.getMotorOutputVoltage();
        double right_voltage = m_rightDrive.getMotorOutputVoltage();

        m_leftDrive.set(
            ControlMode.PercentOutput, 
            Constants.Logistic(steering_adjust)
        );
        m_rightDrive.set(
            ControlMode.PercentOutput, 
            Constants.Logistic(steering_adjust)
        );

    }

    public void chase() {
        // Detect the distance between the robot and target
        // and move towards it

        System.out.println("Positioning robot...");

        // Proportionality constants
        // Must be negative
        final double k_dist = -0.025;

        // Calculated values
        double dist = getLimelightDistance();
        double d_dist = idealDistance - dist;
        double speed = k_dist * d_dist;

        // Drive
        if ( tv.getDouble(0) == 1 ) {
            Drive.directDrive(speed, -speed);    
        }

    }

    public void aim() {
        // Activate precise rotation mechanism to lock 
        // onto the detected target

        System.out.println("Aiming robot...");

        // Proportionality constants
        // Must be negative
        final float k_aim = -0.1f;

        // Cache sensor data for future processing
        float tx = (float)getx();

        float ty = (float)gety();

        // Constants
        float d_heading = -tx;

        
        // Set steer
        float steer = 0.0f;
        if ( d_heading >= 1.0 ) { 
            steer = k_aim * d_heading - Constants.min;
        } else if ( d_heading <= -1.0 ) {
            steer = k_aim * d_heading + Constants.min;
        }

        float distance_error = -ty;
        
        final float k_dist = -0.1f;

        float distance_adjust = k_dist * distance_error;
     
        // Steer robot
        if ( tv.getDouble(0) == 1 ) {
            if(ta.getDouble(0) >= 7){
                Drive.directDrive((steer -= distance_adjust), (steer += distance_adjust));  
                //System.out.println(ta.getDouble(0.0));
                System.out.println("detecting!");
            }
        }

    }

    public void seek() {
        // Uses the limelight 
        // to identify the target 
        // and aim automatically

        System.out.println("Seeking target...");

        // Default spin speed (for when target is not found)
        // Doesn't work below 0.5
        double steer = 0.75;

        if ( tv.getDouble(0) == 0 ) {
            Drive.directDrive(steer, steer);
        } else { aim(); }

    }
    
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.depreciated;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class uses a Pixy2 set on line follow mode to follow lines with the
 * robot.
 * 
 * Setup: -Use the Pixy 2 PixyMon program to configure your Pixy to the mode
 * line follow. -Use the Pixy_Line_Follower Arduino program on a compatable
 * arduino. -Hook the Pixy2 to the Arduino over the included SPI connection.
 * -Hook the Arduino to the RoboRio with I2C.
 * 
 * Instructions: -Run the constructor, PixyLineFollow() in the robotInit()
 * method. -Run arduinoRead() method every loop. -Either get the error and use
 * it yourself or use a provided drive function to follow a line.
 * 
 * @author Max Green
 */
public class PixyLineFollow {

    /**
     * m_error is calculated using the X1 from the primary vector returned by the
     * Pixy and finding the distance from the center of the screen. Negative values
     * indicate that the robot must turn left and positive values indicate that the
     * robot must turn right. The farther from zero the error is the more the robot
     * must turn.
     */
    public int m_error;

    // The SerialPort that the Arduino is on
    private SerialPort m_arduino;

    /**
     * This constructor initializes the class with the standard USB port and a basic
     * baud rate of 9600. It sets the BufferSize to 1.
     */
    public PixyLineFollow() {
        m_arduino = new SerialPort(9600, Port.kUSB);
        m_arduino.setReadBufferSize(1);
    }

    /**
     * This constructor initializes the class with the standard USB port and a
     * custom baud rate. It sets the BufferSize to 1.
     * 
     * @param baudRate The baud rate that matches the Arduino baud rate.
     */
    public PixyLineFollow(int baudRate) {
        m_arduino = new SerialPort(baudRate, Port.kUSB);
        m_arduino.setReadBufferSize(1);
    }

    /**
     * This constructor initializes with a custom SerialPort. It sets the BufferSize
     * to 1.
     * 
     * @param arduinoPort The SerialPort that the Arduino is connected to.
     */
    public PixyLineFollow(SerialPort arduinoPort) {
        m_arduino = arduinoPort;
        m_arduino.setReadBufferSize(1);
    }

    /**
     * This constructor initializes with a custom SerialPort and buffer size.
     * 
     * @param arduinoPort The SerialPort that the Arduino is connected to.
     * @param bufferSize  The abount of bytes the buffer will save.
     */
    public PixyLineFollow(SerialPort arduinoPort, int bufferSize) {
        m_arduino = arduinoPort;
        m_arduino.setReadBufferSize(bufferSize);
    }

    /**
     * This method will read the bytes from the Arduino SerialPort and use the bytes
     * to find and update m_error. If error equals 100 then there is no line to
     * track.
     */
    public void arduinoRead() {
        // Reads all the bytes in the byte string
        byte[] output = m_arduino.read(m_arduino.getBytesReceived());

        // Check that there is a byte to recieve
        if (output.length == 0) {
            // If there are no bytes then set m_error to 100 indicating that Line Follow is
            // not ready.
            m_error = 100;
        } else {
            // If there is a byte set m_error to equal the most recent byte.
            m_error = output[output.length - 1];
        }
    }

    /**
     * This method is used to determine if the Pixy 2 is locked on to a line.
     * 
     * @return Returns a boolean that tells if the line a line is being tracked.
     */
    public boolean isTracking() {
        // If error is not equal to 100, the tracking number then the line is being
        // tracked
        if (m_error != 100) {
            return true;
        }
        return false;
    }

    /**
     * A method that uses the m_error to control the robot. This method uses 2
     * TalonSRXs.
     * 
     * @param left  The left TalonSRX
     * @param right The right TalonSRX
     * @param speed The speed that the function should run at. (0 to 1)
     */
    public void lineFollowTalonSRX(TalonSRX left, TalonSRX right, double speed) {
        // Set the base speeds of the function
        double driveRight = speed;
        double driveLeft = speed;

        // Calculate the percent to multiply the slower side by.
        double turnRate = (Math.abs(m_error) * 2 * speed) / 100;

        // Set drive speeds
        if (m_error < 0) {
            driveLeft -= turnRate;
        } else if (m_error > 0) {
            driveRight -= turnRate;
        }

        // Set motors
        left.set(ControlMode.PercentOutput, driveLeft);
        right.set(ControlMode.PercentOutput, driveRight);
    }

    /**
     * A method that uses the m_error to control the robot. This method uses 2 Speed
     * Controllers.
     * 
     * @param left  The left Speed Controller
     * @param right The right Speed Controller
     * @param speed The speed that the function should run at. (0 to 1)
     */
    public void lineFollowTalon(SpeedController left, SpeedController right, double speed) {
        // Set the base speeds of the function
        double driveRight = speed;
        double driveLeft = speed;

        // Calculate the percent to multiply the slower side by.
        double turnRate = (Math.abs(m_error) * 2 * speed) / 100;

        // Set drive speeds
        if (m_error < 0) {
            driveLeft -= turnRate;
        } else if (m_error > 0) {
            driveRight -= turnRate;
        }

        // Set motors
        left.set(driveLeft);
        right.set(driveRight);
    }

    /**
     * A method that uses the m_error to control the robot. This method uses 4 Speed
     * Controllers.
     * 
     * @param leftFront The left front Speed Controller
     * @param leftBack  The left back Speed Controller
     * @param rightBack The right front Speed Controller
     * @param rightBack The right back Speed Controller
     * @param speed     The speed that the function should run at. (0 to 1)
     */
    public void lineFollowTalon(SpeedController leftFront, SpeedController leftBack, SpeedController rightFront,
            SpeedController rightBack, double speed) {
        // Set the base speeds of the function
        double driveRight = speed;
        double driveLeft = speed;

        // Calculate the percent to multiply the slower side by.
        double turnRate = (Math.abs(m_error) * 2 * speed) / 100;

        // Set drive speeds
        if (m_error < 0) {
            driveLeft -= turnRate;
        } else if (m_error > 0) {
            driveRight -= turnRate;
        }

        // Set motors
        leftFront.set(driveLeft);
        leftBack.set(driveLeft);
        rightFront.set(driveRight);
        rightBack.set(driveRight);
    }
}
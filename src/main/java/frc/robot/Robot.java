// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 1;
  private static final int kJoystickPort = 0;
  private static final int kMotorFalconPort = 2;
  private static final int kMotorSparkMaxPort = 1;

  private MotorController m_motorTalon;
  private CANSparkMax m_motorSpark;
  private WPI_TalonFX m_motorFalcon;
  private Joystick m_joystick;
  private double m_dSpeed;

   /**
   * A RelativeEncoder object is constructed using the GetEncoder() method on an 
   * existing CANSparkMax object. The assumed encoder type is the hall effect,
   * or a sensor type and counts per revolution can be passed in to specify
   * a different kind of sensor. Here, it's a quadrature encoder with 4096 CPR.
   */
  private RelativeEncoder m_encoder;

  @Override
  public void robotInit() {
     // initialize TALON SRX with CAN ID
    m_motorTalon = new WPI_TalonSRX(kMotorPort);
     // initialize SPARK MAX with CAN ID
    m_motorSpark = new  CANSparkMax(kMotorSparkMaxPort, MotorType.kBrushless);
     // initialize FALCON FX 500 with CAN ID
    m_motorFalcon = new WPI_TalonFX(kMotorFalconPort, "rio");
    //Joystick variable
    m_joystick = new Joystick(kJoystickPort);
    //set encoder constructor of the spark max
    m_encoder = m_motorSpark.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
     /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motorSpark.restoreFactoryDefaults();
    //configuration of the Falcon FX 500
    m_motorFalcon.configFactoryDefault();
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Joystick Y",m_joystick.getY());
    SmartDashboard.putNumber("Joystick X",m_joystick.getX());
    m_dSpeed = SmartDashboard.getNumber("Falcon Speed", 0);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
     // periodically read voltage, temperature, and applied output and publish to SmartDashboard
     SmartDashboard.putNumber("Voltage", m_motorSpark.getBusVoltage());
     SmartDashboard.putNumber("Temperature", m_motorSpark.getMotorTemperature());
     SmartDashboard.putNumber("Output", m_motorSpark.getAppliedOutput());
  }

  @Override
  public void teleopPeriodic() {
    m_motorTalon.set(m_joystick.getX());
    m_motorSpark.set(m_joystick.getY());
    m_motorFalcon.set(ControlMode.PercentOutput, m_dSpeed);
  }
}

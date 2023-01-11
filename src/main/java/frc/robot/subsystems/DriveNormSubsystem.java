// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveNormSubsystemConst;

public class DriveNormSubsystem extends SubsystemBase {
  /** Creates a new DriveNormSubsystem. */

  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_rightMotor1;
  private SparkMaxPIDController m_leftPID;
  private SparkMaxPIDController m_rightPID;

  private DifferentialDrive m_differentialDrive;

  //private SparkMaxRelativeEncoder m_leftEncoder1;
  //private SparkMaxRelativeEncoder m_rightEncoder1;

  private SimpleMotorFeedforward m_feedForward;
  private DifferentialDriveVoltageConstraint m_autoVoltageConstraint;
  private TrajectoryConfig m_trajectoryConfig;

  private DifferentialDriveOdometry m_diffOdometry;

  private double m_driveDistance;

  private boolean m_bool1 = false;

  public DriveNormSubsystem() {
    m_leftMotor1 = new CANSparkMax(DriveNormSubsystemConst.kLEFT_MOTOR1, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(DriveNormSubsystemConst.kRIGHT_MOTOR1, MotorType.kBrushless);
    resetMotor(m_leftMotor1);
    resetMotor(m_rightMotor1);
    m_leftMotor1.setInverted(DriveNormSubsystemConst.kINVERTLEFT);
    m_rightMotor1.setInverted(DriveNormSubsystemConst.kINVERTRIGHT);

    m_leftPID = m_leftMotor1.getPIDController();
    m_rightPID = m_rightMotor1.getPIDController();

    setTrapPID(m_leftPID);
    setTrapPID(m_rightPID);

    m_leftMotor1.getEncoder().setPositionConversionFactor(DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);
    m_rightMotor1.getEncoder().setPositionConversionFactor(DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);

    m_differentialDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    m_feedForward = new SimpleMotorFeedforward(
      DriveNormSubsystemConst.kS_VOLTS,
      DriveNormSubsystemConst.kV_VOLT_SECOND_PER_METER,
      DriveNormSubsystemConst.kA_VOLT_SEONDS_SQUARED_PER_METER);

    //System.out.printf("Encoder dist per pulse %.4f", DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);

    m_autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      m_feedForward,
      DriveNormSubsystemConst.kDRIVE_KINEMATICS,
      DriveNormSubsystemConst.kMAX_VOLTAGE);

    m_trajectoryConfig = new TrajectoryConfig(
      DriveNormSubsystemConst.kMAX_SPEED_METERS_PER_SECOND,
      DriveNormSubsystemConst.kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
      .setKinematics(DriveNormSubsystemConst.kDRIVE_KINEMATICS)
      .addConstraint(m_autoVoltageConstraint);

    m_diffOdometry = new DifferentialDriveOdometry(new Rotation2d(), m_driveDistance, m_driveDistance);
  }

  private void resetMotor(CANSparkMax mtrCntl) {
    mtrCntl.restoreFactoryDefaults();
    mtrCntl.setOpenLoopRampRate(DriveNormSubsystemConst.kRAMP_RATE);
    mtrCntl.setSmartCurrentLimit(DriveNormSubsystemConst.kCURRENT_LIMT);
    mtrCntl.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftPosition", m_leftMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("RightPosition", m_rightMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("TargetPosition", m_driveDistance);

    m_diffOdometry.update(new Rotation2d(), m_leftMotor1.getEncoder().getPosition(), m_rightMotor1.getEncoder().getPosition());
  }

  public Pose2d getPos() {
    return m_diffOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftMotor1.getEncoder().getVelocity(), m_rightMotor1.getEncoder().getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_diffOdometry.resetPosition(new Rotation2d(0), m_leftMotor1.getEncoder().getPosition(), m_rightMotor1.getEncoder().getPosition(), pose);
  }

  public void arcadeDrive(double forward, double rotation) {
    m_differentialDrive.feed();
    m_differentialDrive.arcadeDrive(forward, rotation);
  }

  public void tankDriveVolts(double leftVolt, double rightVolt) {
    m_leftMotor1.setVoltage(leftVolt);
    m_rightMotor1.setVoltage(rightVolt);
    m_differentialDrive.feed();
  }

  // for trapezoidal drive
  public void resetTrapezoidal() {
    setTrapPID(m_leftPID);
    setTrapPID(m_rightPID);
    resetEncoders();
  }

  private void setTrapPID(SparkMaxPIDController mtrPID) {
    mtrPID.setP(DriveNormSubsystemConst.kCONTROL_P);
    mtrPID.setI(DriveNormSubsystemConst.kCONTROL_I);
    mtrPID.setD(DriveNormSubsystemConst.kCONTROL_D);
    mtrPID.setIZone(DriveNormSubsystemConst.kCONTROL_IZONE);
    //mtrPID.setFF(DriveNormSubsystemConst.kCONTROL_FF);
    mtrPID.setOutputRange(DriveNormSubsystemConst.kCONTROL_MIN_OUT, DriveNormSubsystemConst.kCONTROL_MAX_OUT);
  }

  public void setDriveStates(TrapezoidProfile.State leftState, TrapezoidProfile.State rightState) {
    m_leftPID.setReference(
      leftState.position,
      ControlType.kPosition, 
      0,
      m_feedForward.calculate(leftState.velocity));
    m_rightPID.setReference(
      rightState.position, 
      ControlType.kPosition,
      0, 
      m_feedForward.calculate(rightState.velocity));
  }

  // for Smart Motion pid controller drive distance

  public void resetSmartMotion() {
    setSmartMotPID(m_leftPID);
    setSmartMotPID(m_rightPID);
    //resetEncoders();
  }

  private void setSmartMotPID(SparkMaxPIDController mtrPID) {
    mtrPID.setP(DriveNormSubsystemConst.kSMARTMOT_P);
    mtrPID.setI(DriveNormSubsystemConst.kSMARTMOT_I);
    mtrPID.setD(DriveNormSubsystemConst.kSMARTMOT_D);
    mtrPID.setIZone(DriveNormSubsystemConst.kSMARTMOT_IZONE);
    mtrPID.setFF(DriveNormSubsystemConst.kSMARTMOT_FF);
    mtrPID.setOutputRange(DriveNormSubsystemConst.kSMARTMOT_MIN_OUT, DriveNormSubsystemConst.kSMARTMOT_MAX_OUT);
    int smartMotionSlot = 0;
    mtrPID.setSmartMotionMaxVelocity(DriveNormSubsystemConst.kSMARTMOT_MAX_V, smartMotionSlot);
    mtrPID.setSmartMotionMinOutputVelocity(DriveNormSubsystemConst.kSMARTMOT_MIN_V, smartMotionSlot);
    mtrPID.setSmartMotionMaxAccel(DriveNormSubsystemConst.kSMARTMOT_MAX_ACC, smartMotionSlot);
    mtrPID.setSmartMotionAllowedClosedLoopError(DriveNormSubsystemConst.kSMARTMOT_ALLOWED_ERR, smartMotionSlot);
  }
  
  public void initDriveController(double distance) {

    resetSmartMotion();
    //double encoderDistance = distance / DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M;
    m_leftPID.setReference(distance, ControlType.kSmartMotion);
    m_rightPID.setReference(distance, ControlType.kSmartMotion);
    m_driveDistance = distance;
  }

  public void endDriveController() {
    m_leftMotor1.stopMotor();
    m_rightMotor1.stopMotor();
  }

  public boolean isDriveAtSetpoint() {
    boolean leftOnTarget = Math.abs(m_driveDistance - m_leftMotor1.getEncoder().getPosition()) <= 0.1;
    boolean rightOnTarget = Math.abs(m_driveDistance - m_rightMotor1.getEncoder().getPosition()) <= 0.1;
    boolean comboStopped = false; //((Math.abs(m_leftMotor1.getEncoder().getVelocity()) + (Math.abs(m_rightMotor1.getEncoder().getVelocity()))) < 0.05);
    return (leftOnTarget && rightOnTarget || comboStopped);
  }

  public TrajectoryConfig getTrajConfig() {
    return m_trajectoryConfig;
  }

  public SimpleMotorFeedforward getSmpFeedFwd() {
    return m_feedForward;
  }

  /*
  public PIDController getLeftPidController() {
    return m_leftPID;
  }
  */

  public void resetEncoders() {
    m_leftMotor1.getEncoder().setPosition(0);
    m_rightMotor1.getEncoder().setPosition(0);
  }

  public void setBool1(boolean bool) {
    m_bool1 = bool;
    //System.out.println("DrivesetBool" + m_bool1);
  }

  public boolean isBool1() {
    //System.out.println("DriveisBool" + m_bool1);
    return m_bool1;
  } 
}

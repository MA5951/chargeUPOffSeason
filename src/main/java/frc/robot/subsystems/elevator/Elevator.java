// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Elevator extends SubsystemBase implements 
  DefaultInternallyControlledSubsystem{
  /** Creates a new Elevator. */
  private CANSparkMax master;
  private CANSparkMax slave;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private MAShuffleboard board;
  private pidControllerGainSupplier pidSupplier;
  private double setPoint;
  private static Elevator instance;

  private Elevator() {
    master = new CANSparkMax(PortMap.Elevator.masterMotorID, MotorType.kBrushless);
    encoder = master.getAlternateEncoder(
      SparkMaxAlternateEncoder.Type.kQuadrature, ElevatorConstance.kCPR);
    encoder.setPositionConversionFactor(
      ElevatorConstance.positionConversionFactor
    );

    master.setIdleMode(IdleMode.kCoast);
    slave.setIdleMode(IdleMode.kCoast);

    pidController = master.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ElevatorConstance.kP);
    pidController.setI(ElevatorConstance.kI);
    pidController.setD(ElevatorConstance.kD);

    slave.follow(master);

    board = new MAShuffleboard("Elevator");
    pidSupplier = board.getPidControllerGainSupplier(
      ElevatorConstance.kP, ElevatorConstance.kI, ElevatorConstance.kD);
  }

  public double getFeed() {
    return ElevatorConstance.kG;
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition,
      0, getFeed(), ArbFFUnits.kPercentOut);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint) <=
     ElevatorConstance.positionConversionFactor;
  }

  @Override
  public void setVoltage(double voltage) {
    master.set(voltage / 12);
  }

  @Override
  public boolean canMove() {
    return setPoint > ElevatorConstance.minPose
      && setPoint < ElevatorConstance.maxPose;
  }

  @Override
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public double getSetPoint() {
    return setPoint;
  }

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  @Override
  public void periodic() {
    board.addNum("pose", encoder.getPosition());
    pidController.setP(pidSupplier.getKP());
    pidController.setI(pidController.getI());
    pidController.setD(pidSupplier.getKD());
  }
}

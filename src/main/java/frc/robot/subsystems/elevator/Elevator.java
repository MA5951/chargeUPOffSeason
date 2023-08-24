// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Elevator extends SubsystemBase implements 
  DefaultInternallyControlledSubsystem{
  /** Creates a new Elevator. */
  private CANSparkMax master;
  private CANSparkMax slave;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private MAShuffleboard board;
 // private pidControllerGainSupplier pidSupplier;
  private double setPoint = 0;
  private static Elevator instance;
  // private DigitalInput uppdeHalleffect;
  // private DigitalInput lowerHalleffect;


  private Elevator() {
    master = new CANSparkMax(PortMap.Elevator.masterMotorID, MotorType.kBrushless);
    slave = new CANSparkMax(PortMap.Elevator.slaveMotorID, MotorType.kBrushless);

    master.setIdleMode(IdleMode.kBrake);
    slave.setIdleMode(IdleMode.kBrake);

    encoder = master.getEncoder();
      //SparkMaxAlternateEncoder.Type.kQuadrature, ElevatorConstance.kCPR);
    encoder.setPositionConversionFactor(
      ElevatorConstance.positionConversionFactor
    );
    encoder.setVelocityConversionFactor(
      ElevatorConstance.positionConversionFactor / 60
    );
    encoder.setPosition(0);

    master.setClosedLoopRampRate(ElevatorConstance.closedLoopRampRate);
    master.setOpenLoopRampRate(0);

    pidController = master.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ElevatorConstance.kP);
    pidController.setI(ElevatorConstance.kI);
    pidController.setD(ElevatorConstance.kD);

    slave.follow(master, true);

    board = new MAShuffleboard("Elevator");
    // pidSupplier = board.getPidControllerGainSupplier(
    //   ElevatorConstance.kP, ElevatorConstance.kI, ElevatorConstance.kD);
  }

  public void resetPose(double pose) {
    encoder.setPosition(pose);
  }

  public double getExtension() {
    return encoder.getPosition();
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(getExtension() - setPoint) <=
     ElevatorConstance.tolerance;
  }

  @Override
  public void setVoltage(double voltage) {
    master.set(voltage / 12);
  }

  @Override
  public boolean canMove() {
    return setPoint >= ElevatorConstance.minPose
      && setPoint <= ElevatorConstance.maxPose
      && SwerveDrivetrainSubsystem.getInstance().getAccelerationX()
      <= SwerveConstants.maxAccelerationForOpenElevator;
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

  public double getCurrent() {
    return master.getOutputCurrent();
  }

  @Override
  public void periodic() {
    board.addNum("pose", getExtension());
    board.addNum("v", master.getBusVoltage());
    board.addNum("A", master.getOutputCurrent());
    board.addNum("setpoint", setPoint);
    board.addBoolean("can move", canMove());    
  }
}

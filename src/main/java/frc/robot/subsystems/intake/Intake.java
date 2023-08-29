package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.MotorSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;

public class Intake extends SubsystemBase implements MotorSubsystem{

    private static Intake intake;

    private CANSparkMax intakeMotor;
    private DigitalInput sensor;

    private MAShuffleboard board;

    private boolean coneInIntake = false;

    private boolean ignoreCurrent = false;
    private boolean ignoreSensor = false;

    private boolean cubeInIntake = false;

    private Intake() {
        intakeMotor = new CANSparkMax(PortMap.Intake.intakeMotorID, MotorType.kBrushless);

        sensor = new DigitalInput(PortMap.Intake.sensorID);

        board = new MAShuffleboard("Intake");

        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public boolean getSensor() {
        return !sensor.get();
    }

    public double getMotorCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public boolean isConeIn() {
        return coneInIntake;
    }
    
    public void setConeState(boolean state) {
        coneInIntake = state;
    }

    public void setCubeState(boolean state) {
        cubeInIntake = state;
    }

    public void removeGamePieces() {
        setConeState(false);
        setCubeState(false);
        setIgnoreSensor(false);
    }

    public boolean isCubeIn() {
        return cubeInIntake;
    }

    public boolean isPieceInIntake() {
        return isCubeIn() || isConeIn();
    }

    @Override
    public boolean canMove() {
        return true;
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMotor.set(voltage / 12);
    }

    public void setIgnoreCurrent(boolean ignoreCurrent) {
        this.ignoreCurrent = ignoreCurrent;
    }

    public void setIgnoreSensor(boolean ignoreSensor) {
        this.ignoreSensor = ignoreSensor;
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();  
        }
        return intake;
    }

    @Override
    public void periodic() {
        board.addBoolean("Is Piece in Intake", isPieceInIntake());
        board.addBoolean("Is Cone in Intake", isConeIn());
        board.addBoolean("Is cube in Intake", isCubeIn());
        board.addBoolean("is CLimitSwitch", getSensor());
        board.addNum("Intake Current", getMotorCurrent());
        
        if (getMotorCurrent() > IntakeConstance.currentAmpThreshold && !ignoreCurrent) {
            setConeState(true);
        }

        if (getSensor() && !ignoreSensor){
            setCubeState(true);
        }
    }
}

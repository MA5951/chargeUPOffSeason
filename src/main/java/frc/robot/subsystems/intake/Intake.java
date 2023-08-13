package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.MotorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;

public class Intake extends SubsystemBase implements MotorSubsystem{

    private static Intake intake;

    private CANSparkMax intakeMotor;

    private MAShuffleboard board;

    private boolean ignoreCurrent = false;

    private boolean isGamePiceIn = false;
    private boolean isConeIn = false;

    private double currentAmpThreshold = IntakeConstance.currentAmpThresholdCude;

    private Intake() {
        intakeMotor = new CANSparkMax(PortMap.Intake.intakeMotorID, MotorType.kBrushless);

        board = new MAShuffleboard("Intake");

        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCurrentAmpThreshold (double currentAmpThreshold) {
        this.currentAmpThreshold = currentAmpThreshold;
    }

    public double getMotorCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public void setIsGamePiceIn(boolean isGamePiceIn) {
        this.isGamePiceIn = isGamePiceIn;
    }

    public void removeGamePieces() {
        setIsGamePiceIn(false);
        setIsConeIn(false);
    }

    public boolean isConeIn() {
        return isConeIn;
    }

    public void setIsConeIn(boolean isConeIn) {
        this.isConeIn = isConeIn;
    }

    public boolean isCubeIn() {
        return isGamePiceIn && !isConeIn;
    }

    public boolean isPieceInIntake() {
        return isGamePiceIn;
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
        board.addBoolean("is Cube In intake", isCubeIn());
        board.addNum("Intake Current", getMotorCurrent());
        
        if (getMotorCurrent() > currentAmpThreshold && !ignoreCurrent) {
            setIsGamePiceIn(true);
            if (currentAmpThreshold == IntakeConstance.currentAmpThresholdCone) {
                setIsConeIn(true);
            } else {
                setIsConeIn(false);
            }
        }
    }
}

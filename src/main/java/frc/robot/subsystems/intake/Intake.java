package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.MotorSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;

public class Intake extends SubsystemBase implements MotorSubsystem{

    private static Intake intake;

    private CANSparkMax intakeMotor;
    private DigitalInput limitSwitch;

    private MAShuffleboard board;

    private boolean coneInIntake = false;

    private Intake() {
        intakeMotor = new CANSparkMax(PortMap.Intake.intakeMotorID, MotorType.kBrushless);

        limitSwitch = new DigitalInput(PortMap.Intake.intakeLimitSwitchID);

        board = new MAShuffleboard("Intake");
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public double getMotorCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public boolean isConeIn() {
        if (getMotorCurrent() > IntakeConstance.currentAmpThreshold) {
            coneInIntake = true;
        } else {
            coneInIntake = false;
        }

        return coneInIntake;
    }
    
    public void setConeState(boolean state) {
        coneInIntake = state;
    }

    public boolean isPieceInIntake() {
        return getLimitSwitch() || isConeIn();
    }

    public void setPower(double power){
        intakeMotor.set(power);
    }

    @Override
    public boolean canMove() {
        return true;
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMotor.set(voltage);
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
    }
}

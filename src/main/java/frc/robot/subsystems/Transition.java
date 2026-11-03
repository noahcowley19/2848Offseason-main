package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.HopperConstants.kForwardBeltMotorID;
import static frc.robot.Constants.HopperConstants.kSidewaysBeltMotorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transition extends SubsystemBase{


    private final TalonFX sidewaysMotor;
    private final TalonFX forwardMotor;


    public Transition()
    {
        sidewaysMotor = new TalonFX(kSidewaysBeltMotorID);
        forwardMotor = new TalonFX(kForwardBeltMotorID);
        TalonFXConfiguration beltConfig = new TalonFXConfiguration();

        beltConfig.Slot0.kS = 0;
        beltConfig.Slot0.kV = 0.12;
        beltConfig.Slot0.kP = 0;
        beltConfig.Slot0.kI = 0;
        beltConfig.Slot0.kD = 0;
        beltConfig.Slot0.kA = 0;

        beltConfig.Voltage
            .withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));
        beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        sidewaysMotor.getConfigurator().apply(beltConfig);
        forwardMotor.getConfigurator().apply(beltConfig);
        sidewaysMotor.setControl(new Follower(forwardMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setSpeed(double speed)
    {
        sidewaysMotor.set(speed);
    }
    public void stop()
    {
        sidewaysMotor.set(0);
    }

    public void periodic()
    {
        SmartDashboard.putData(this);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Velocity Output", () -> sidewaysMotor.getVelocity().getValueAsDouble(), null);
       builder.addDoubleProperty("Voltage", () -> sidewaysMotor.getMotorVoltage().getValueAsDouble(), null);

    }
}
    

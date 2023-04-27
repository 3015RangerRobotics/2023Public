package frc.lib.subsystem.selfcheck;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.CANcoder;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingCANCoderPro implements SelfChecking {
  private final String label;
  private final StatusSignalValue<Boolean> hardwareFaultSignal;
  private final StatusSignalValue<Boolean> bootEnabledSignal;
  private final StatusSignalValue<Boolean> badMagnetSignal;
  private final StatusSignalValue<Boolean> undervoltageSignal;

  public SelfCheckingCANCoderPro(String label, CANcoder canCoder) {
    this.label = label;

    this.hardwareFaultSignal = canCoder.getFault_Hardware();
    this.bootEnabledSignal = canCoder.getFault_BootDuringEnable();
    this.badMagnetSignal = canCoder.getFault_BadMagnet();
    this.undervoltageSignal = canCoder.getFault_Undervoltage();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (hardwareFaultSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (bootEnabledSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (badMagnetSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Magnet too weak", label)));
    }
    if (undervoltageSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Under voltage", label)));
    }

    return faults;
  }
}

package frc.lib.subsystem.selfcheck;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.TalonFX;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingTalonFXPro implements SelfChecking {
  private final String label;
  private final StatusSignalValue<Integer> firmwareVersionSignal;
  private final StatusSignalValue<Boolean> hardwareFaultSignal;
  private final StatusSignalValue<Boolean> bootEnabledSignal;
  private final StatusSignalValue<Boolean> deviceTempSignal;
  private final StatusSignalValue<Boolean> procTempSignal;

  public SelfCheckingTalonFXPro(String label, TalonFX talon) {
    this.label = label;

    firmwareVersionSignal = talon.getVersion();
    hardwareFaultSignal = talon.getFault_Hardware();
    bootEnabledSignal = talon.getFault_BootDuringEnable();
    deviceTempSignal = talon.getFault_DeviceTemp();
    procTempSignal = talon.getFault_ProcTemp();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (firmwareVersionSignal.refresh().getError() != StatusCode.OK) {
      faults.add(new SubsystemFault(String.format("[%s]: No communication with device", label)));
    }
    if (hardwareFaultSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (bootEnabledSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (deviceTempSignal.refresh().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device temperature too high", label), true));
    }
    if (procTempSignal.refresh().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Processor temperature too high", label), true));
    }

    return faults;
  }
}

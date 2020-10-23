model VariableVoltageSource
  extends Modelica.Electrical.Analog.Interfaces.VoltageSource;
  input Modelica.Blocks.Interfaces.RealInput voltage; 
equation
  v = voltage;
end VariableVoltageSource;
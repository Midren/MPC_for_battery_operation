model ChargeDependentCapacitor
  extends Modelica.Electrical.Analog.Basic.VariableCapacitor;
  Modelica.Blocks.Interfaces.RealInput SoC;
  parameter Modelica.SIunits.Capacitance C_0;
  parameter Real k1;
  parameter Real k2;
  
equation
  C = C_0 + k1*exp(k2*SoC);

end ChargeDependentCapacitor;
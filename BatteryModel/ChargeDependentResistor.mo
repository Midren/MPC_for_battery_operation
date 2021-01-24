model ChargeDependentResistor
  extends Modelica.Electrical.Analog.Basic.VariableResistor;
  Modelica.Blocks.Interfaces.RealInput SoC;
  parameter Modelica.SIunits.Resistance R_0;
  parameter Real k1;
  parameter Real k2;
  
equation
  R = R_0 + k1*exp(k2*SoC);

end ChargeDependentResistor;
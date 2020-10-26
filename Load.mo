model Load
  extends Modelica.Electrical.Analog.Basic.VariableResistor;
  parameter Modelica.SIunits.Current I_req;
equation
  R = v/I_req;
end Load;
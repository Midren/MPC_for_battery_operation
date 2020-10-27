model VariableVoltageSource
  extends Modelica.Electrical.Analog.Interfaces.VoltageSource;
  Modelica.Blocks.Interfaces.RealInput
            voltage "Connector of Real input signals" annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100})));
equation
  v = v+voltage;
  annotation (uses(Modelica(version="3.2.3")));
end VariableVoltageSource;

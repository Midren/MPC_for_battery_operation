model VariableSource "Generate signal of type Real"
  Real height "Height of step"
  annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Step.png"));
  extends Modelica.Blocks.Interfaces.SignalSource;
equation
  y = offset + (if time < startTime then 0 else height);
end VariableSource;
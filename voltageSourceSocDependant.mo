model voltageSourceSocDependant"Interface for voltage sources"
  extends Modelica.Electrical.Analog.Interfaces.TwoPin;
  parameter String SocToULookupTableFileName;
  
  Modelica.Electrical.Analog.Sensors.VoltageSensor U_ocv annotation(
    Placement(visible = true, transformation(origin = {-2, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Tables.CombiTable1D SOC_to_U_bat(fileName = SocToULookupTableFileName, tableName = "tab", tableOnFile = true) annotation(
    Placement(visible = true, transformation(origin = {-12, 74}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum voltageChange(k = {1, -1}, nin = 2) annotation(
    Placement(visible = true, transformation(origin = {-62, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  VariableVoltageSource signalVoltage(v(start = 4.1)) annotation(
    Placement(visible = true, transformation(origin = {-2, -46}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput SoC annotation(
    Placement(visible = true, transformation(origin = {0, 122}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 122}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
equation
  
  connect(U_ocv.v, voltageChange.u[1]) annotation(
    Line(points = {{-2, 45}, {-48, 45}, {-48, 60}, {-54, 60}}, color = {0, 0, 127}));
  connect(voltageChange.y, signalVoltage.voltage) annotation(
    Line(points = {{-69, 60}, {-88.25, 60}, {-88.25, 18}, {-2.5625, 18}, {-2.5625, -36}, {-2, -36}}, color = {0, 0, 127}));
  connect(SoC, SOC_to_U_bat.u[1]) annotation(
    Line(points = {{0, 122}, {0, 74}}, color = {0, 0, 127}));
  connect(voltageChange.u[2], SOC_to_U_bat.y[2]) annotation(
    Line(points = {{-54, 60}, {-49, 60}, {-49, 74}, {-23, 74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(U_ocv.p, signalVoltage.p) annotation(
    Line(points = {{8, 34}, {8, -6}, {-12, -6}, {-12, -46}}, color = {0, 0, 255}));
  connect(U_ocv.n, signalVoltage.n) annotation(
    Line(points = {{-12, 34}, {-12, -6}, {8, -6}, {8, -46}}, color = {0, 0, 255}));
  connect(signalVoltage.p, p) annotation(
    Line(points = {{-12, -46}, {-60, -46}, {-60, 0}, {-100, 0}, {-100, 0}}, color = {0, 0, 255}));
  connect(signalVoltage.n, n) annotation(
    Line(points = {{8, -46}, {40, -46}, {40, 0}, {100, 0}, {100, 0}}, color = {0, 0, 255}));
  annotation(
    uses(Modelica(version = "3.2.3")));
end voltageSourceSocDependant;
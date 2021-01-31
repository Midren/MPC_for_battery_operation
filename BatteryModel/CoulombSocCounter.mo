model CoulombSocCounter
  Modelica.Blocks.Interfaces.RealInput
            I "Connector of Real input signals" annotation (Placement(
        visible = true, transformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 270), iconTransformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));     
  Modelica.Blocks.Sources.Constant SOC_init(k = 1) annotation(
    Placement(visible = true, transformation(origin = {63, -21}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1) annotation(
    Placement(visible = true, transformation(origin = {63, 23}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
  Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2) annotation(
    Placement(visible = true, transformation(origin = {-26, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SoC annotation(
    Placement(visible = true, transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Nonlinear.Limiter socLimiter(limitsAtInit = true, strict = true, uMax = 1, uMin = 0.012)  annotation(
    Placement(visible = true, transformation(origin = {-60, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Q_us annotation(
    Placement(visible = true, transformation(origin = {-40, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-40, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {16, 18}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
equation
  connect(SOC_init.y, Sum.u[1]) annotation(
    Line(points = {{51, -21}, {10, -21}, {10, -10}, {-14, -10}}, color = {0, 0, 127}));
  connect(I, integrator.u) annotation(
    Line(points = {{60, 100}, {59, 100}, {59, 66}, {82, 66}, {82, 23}, {76, 23}}, color = {0, 0, 127}));
  connect(Sum.y, socLimiter.u) annotation(
    Line(points = {{-37, -10}, {-48, -10}}, color = {0, 0, 127}));
  connect(socLimiter.y, SoC) annotation(
    Line(points = {{-71, -10}, {-76, -10}, {-76, -90}, {0, -90}, {0, -110}}, color = {0, 0, 127}));
  connect(Q_us, division.u2) annotation(
    Line(points = {{-40, 100}, {-40, 62}, {36, 62}, {36, 12}, {28, 12}}, color = {0, 0, 127}));
  connect(integrator.y, division.u1) annotation(
    Line(points = {{51, 23}, {40, 23}, {40, 24}, {28, 24}}, color = {0, 0, 127}));
  connect(division.y, Sum.u[2]) annotation(
    Line(points = {{5, 18}, {-4, 18}, {-4, -10}, {-14, -10}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.3")));
end CoulombSocCounter;
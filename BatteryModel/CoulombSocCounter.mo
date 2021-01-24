model CoulombSocCounter
  Modelica.Blocks.Interfaces.RealInput
            I "Connector of Real input signals" annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100})));     
  parameter Real C_bat "Nominal capacity in Ah";
  Modelica.Blocks.Sources.Constant SOC_init(k = 1) annotation(
    Placement(visible = true, transformation(origin = {63, -21}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1 / C_bat) annotation(
    Placement(visible = true, transformation(origin = {63, 17}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant minSoc(k = 0) annotation(
    Placement(visible = true, transformation(origin = {-6, -32}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant maxSoc(k = 1) annotation(
    Placement(visible = true, transformation(origin = {-6, 36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2) annotation(
    Placement(visible = true, transformation(origin = {-6, -2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter socLimiter(homotopyType = Modelica.Blocks.Types.VariableLimiterHomotopy.Linear,limitsAtInit = true, strict = true)  annotation(
    Placement(visible = true, transformation(origin = { -36, -2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SoC annotation(
    Placement(visible = true, transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(maxSoc.y, socLimiter.limit1) annotation(
    Line(points = {{-17, 36}, {-24, 36}, {-24, 6}}, color = {0, 0, 127}));
  connect(Sum.y, socLimiter.u) annotation(
    Line(points = {{-17, -2}, {-24, -2}}, color = {0, 0, 127}));
  connect(SOC_init.y, Sum.u[1]) annotation(
    Line(points = {{51, -21}, {10, -21}, {10, -2}, {6, -2}}, color = {0, 0, 127}));
  connect(minSoc.y, socLimiter.limit2) annotation(
    Line(points = {{-17, -32}, {-24, -32}, {-24, -10}}, color = {0, 0, 127}));
  connect(integrator.y, Sum.u[2]) annotation(
    Line(points = {{51, 17}, {9.5, 17}, {9.5, -2}, {6, -2}}, color = {0, 0, 127}));
  connect(I, integrator.u) annotation(
    Line(points = {{0, 100}, {7.5, 100}, {7.5, 98}, {1, 98}, {1, 64}, {82, 64}, {82, 18}, {76, 18}}, color = {0, 0, 127}));
  connect(socLimiter.y, SoC) annotation(
    Line(points = {{-46, -2}, {-60, -2}, {-60, -80}, {0, -80}, {0, -110}, {0, -110}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.3")));
end CoulombSocCounter;
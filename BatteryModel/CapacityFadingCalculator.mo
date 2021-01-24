model CapacityFadingCalculator
  parameter Real C_bat "Nominal capacity in Ah";
  Boolean isCharging(start = true);
  Real cycleStartTime(start = 0);
  Real cycleLen;
  Real SoC_avg(start=0);
  Real SoC_dev(start=0);
  parameter Real k_s1;
  parameter Real k_s2;
  parameter Real k_s3;
  parameter Real k_s4;
  Modelica.Blocks.Interfaces.RealInput SoC annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput I annotation(
    Placement(visible = true, transformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Continuous.Integrator socIntegrator(use_reset = true, y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product socSquare annotation(
    Placement(visible = true, transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator socSquareIntegrator(use_reset = true, y_start = 0)  annotation(
    Placement(visible = true, transformation(origin = {-10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator Ah_throughput(use_reset = true, y_start = 0)  annotation(
    Placement(visible = true, transformation(origin = {68, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput capacityFade annotation(
    Placement(visible = true, transformation(origin = {0, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90), iconTransformation(origin = {0, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Blocks.Math.Abs I_abs annotation(
    Placement(visible = true, transformation(origin = {22, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  isCharging = if I < 0 then true else false;
  when isCharging == false then
    cycleLen = time - pre(cycleStartTime);
    SoC_avg = (1/cycleLen) * pre(socIntegrator.y);
    SoC_dev = 2*sqrt((3/cycleLen))*sqrt(pre(socSquareIntegrator.y) - (2*SoC_avg) * pre(socIntegrator.y) + cycleLen*SoC_avg*SoC_avg);
    capacityFade = pre(Ah_throughput.y)*(k_s1*SoC_dev+exp(k_s2*SoC_avg) + k_s3*exp(k_s4*SoC_dev));
    
    cycleStartTime = time;
    socIntegrator.reset = true;
    socSquareIntegrator.reset = true;
    Ah_throughput.reset = true;
  end when;
  connect(SoC, socIntegrator.u) annotation(
    Line(points = {{-100, 0}, {-79, 0}, {-79, 20}, {-62, 20}}, color = {0, 0, 127}));
  connect(SoC, socSquare.u1) annotation(
    Line(points = {{-100, 0}, {-74, 0}, {-74, -14}, {-62, -14}}, color = {0, 0, 127}));
  connect(SoC, socSquare.u2) annotation(
    Line(points = {{-100, 0}, {-74, 0}, {-74, -26}, {-62, -26}}, color = {0, 0, 127}));
  connect(socSquare.y, socSquareIntegrator.u) annotation(
    Line(points = {{-38, -20}, {-22, -20}}, color = {0, 0, 127}));
  connect(I, I_abs.u) annotation(
    Line(points = {{0, 100}, {0, 100}, {0, 50}, {10, 50}, {10, 50}}, color = {0, 0, 127}));
  connect(I_abs.y, Ah_throughput.u) annotation(
    Line(points = {{34, 50}, {56, 50}, {56, 50}, {56, 50}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.3")),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end CapacityFadingCalculator;
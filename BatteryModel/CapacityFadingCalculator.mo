model CapacityFadingCalculator
  parameter Real C_bat "Nominal capacity in Ah";
  parameter Real K_co = 3.66e-5;
  parameter Real K_ex = 0.717;
  parameter Real K_soc = 0.916;
  Boolean isCharging(start = true);
  Real cycleStartTime(start = 0);
  Real cycleLen;
  Real SoC_avg(start=0);
  Real SoC_dev(start=0);
  Real n_m(start=0) "Effective number of throughput cycles";
  Real L(start=0) "Life ageing parameter";
  Real stepCapacityFade(start=0);
  
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
  Modelica.Blocks.Math.Abs I_abs annotation(
    Placement(visible = true, transformation(origin = {22, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Q_us(start=C_bat) annotation(
    Placement(visible = true, transformation(origin = {60, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90), iconTransformation(origin = {60, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput capacityFade annotation(
    Placement(visible = true, transformation(origin = {-60, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90), iconTransformation(origin = {-60, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
equation
  isCharging = if I < 0 then true else false;
  L = 1-Q_us/C_bat;
  when isCharging == false then
    cycleLen = time - pre(cycleStartTime);
    SoC_avg = (1/cycleLen) * pre(socIntegrator.y);
    SoC_dev = 2*sqrt((3/cycleLen))*sqrt(pre(socSquareIntegrator.y) - (2*SoC_avg) * pre(socIntegrator.y) + cycleLen*SoC_avg*SoC_avg);
    n_m = pre(Ah_throughput.y)/(2*C_bat);
    
    stepCapacityFade = K_co*n_m*exp((SoC_dev-1)/(K_ex))*exp(K_soc*((SoC_avg-0.5)/0.25))*(1-pre(L));
    Q_us = pre(Q_us) - stepCapacityFade;
    capacityFade = pre(capacityFade) + stepCapacityFade;
    
    cycleStartTime = time;
    socIntegrator.reset = true;
    socSquareIntegrator.reset = true;
    Ah_throughput.reset = true;
  elsewhen isCharging == true then
    cycleLen = pre(cycleLen);
    SoC_avg = pre(SoC_avg);
    SoC_dev = pre(SoC_dev);
    n_m = pre(n_m);
    
    stepCapacityFade = pre(stepCapacityFade);
    capacityFade = pre(capacityFade);
    
    Q_us = pre(Q_us);
    
    cycleStartTime = pre(cycleStartTime);
    socIntegrator.reset = false;
    socSquareIntegrator.reset = false;
    Ah_throughput.reset = false;
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
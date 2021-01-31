model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
  parameter Real C_bat = 1.1;
  Real SoH(start=1);
  //Real I_req(start = -C_bat);
  // Because we need in seconds
  ChargeDependentResistor R_s(R_0 = 0.07446, k1 = 0.1562, k2 = -24.37) annotation(
    Placement(visible = true, transformation(origin = {-48, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ChargeDependentResistor R_ts(R_0 = 0.04669, k1 = 0.3208, k2 = -29.14) annotation(
    Placement(visible = true, transformation(origin = {-10, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ChargeDependentCapacitor C_ts(C_0 = 703.6, k1 = -752.9, k2 = -13.51) annotation(
    Placement(visible = true, transformation(origin = {-10, 12}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  ChargeDependentResistor R_tl(R_0 = 0.04669, k1 = 6.603, k2 = -29.14) annotation(
    Placement(visible = true, transformation(origin = {30, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ChargeDependentCapacitor C_tl(C_0 = 4475, k1 = -6056, k2 = -27.12) annotation(
    Placement(visible = true, transformation(origin = {30, 12}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.CurrentSensor I_bat annotation(
    Placement(visible = true, transformation(origin = {60, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Electrical.Analog.Basic.Ground GND annotation(
    Placement(visible = true, transformation(origin = {-86, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  CoulombSocCounter coulombSocCounter annotation(
    Placement(visible = true, transformation(origin = {30, -22}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  voltageSourceSocDependant voltageSource(SocToULookupTableFileName = "/home/midren/bachelor/soc_to_u_bat_tookup.txt") annotation(
    Placement(visible = true, transformation(origin = {-86, -22}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  CapacityFadingCalculator capacityFadingCalc(C_bat = C_bat) annotation(
    Placement(visible = true, transformation(origin = {4, -58}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.PulseCurrent pulseLoad(I = 2 * C_bat, offset = -C_bat, period = 2, width = 50)  annotation(
    Placement(visible = true, transformation(origin = {34, -90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
equation
  SoH = 1 - capacityFadingCalc.capacityFade/(0.2*C_bat);
// main circuit
  connect(R_s.p, R_ts.n) annotation(
    Line(points = {{-20, 40}, {-38, 40}, {-38, 40}, {-38, 40}}, color = {0, 0, 255}));
  connect(R_ts.n, C_ts.n) annotation(
    Line(points = {{-20, 40}, {-20, 12}}, color = {0, 0, 255}));
  connect(R_ts.p, C_ts.p) annotation(
    Line(points = {{0, 40}, {0, 12}}, color = {0, 0, 255}));
  connect(R_ts.p, R_tl.n) annotation(
    Line(points = {{0, 40}, {20, 40}, {20, 40}, {20, 40}}, color = {0, 0, 255}));
  connect(R_tl.n, C_tl.n) annotation(
    Line(points = {{20, 40}, {20, 12}}, color = {0, 0, 255}));
  connect(R_tl.p, C_tl.p) annotation(
    Line(points = {{40, 12}, {40, 12}}, color = {0, 0, 255}));
  connect(R_tl.p, I_bat.p) annotation(
    Line(points = {{40, 40}, {50, 40}}, color = {0, 0, 255}));
// Find required voltage
// Find voltage difference and modify signalVoltage
  connect(I_bat.i, coulombSocCounter.I) annotation(
    Line(points = {{60, 30}, {60, -28}, {40, -28}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, C_ts.SoC) annotation(
    Line(points = {{19, -22}, {-10, -22}, {-10, 0}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, C_tl.SoC) annotation(
    Line(points = {{19, -22}, {-10, -22}, {-10, -8}, {30, -8}, {30, 0}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, R_tl.SoC) annotation(
    Line(points = {{19, -22}, {6, -22}, {6, 60}, {30, 60}, {30, 52}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, R_ts.SoC) annotation(
    Line(points = {{19, -22}, {6, -22}, {6, 60}, {-10, 60}, {-10, 52}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, R_s.SoC) annotation(
    Line(points = {{19, -22}, {6, -22}, {6, 60}, {-48, 60}, {-48, 52}}, color = {0, 0, 127}));
  connect(voltageSource.n, GND.p) annotation(
    Line(points = {{-86, -32}, {-86, -90}}, color = {0, 0, 255}));
  connect(voltageSource.p, R_s.n) annotation(
    Line(points = {{-86, -12}, {-86, 40}, {-58, 40}}, color = {0, 0, 255}));
  connect(coulombSocCounter.SoC, voltageSource.SoC) annotation(
    Line(points = {{19, -22}, {-74, -22}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, capacityFadingCalc.SoC) annotation(
    Line(points = {{19, -22}, {19, -32}, {4, -32}, {4, -48}}, color = {0, 0, 127}));
  connect(capacityFadingCalc.I, I_bat.i) annotation(
    Line(points = {{14, -58}, {60, -58}, {60, 30}}, color = {0, 0, 127}));
  connect(I_bat.n, pulseLoad.p) annotation(
    Line(points = {{70, 40}, {80, 40}, {80, -90}, {44, -90}}, color = {0, 0, 255}));
  connect(pulseLoad.n, GND.p) annotation(
    Line(points = {{24, -90}, {-86, -90}, {-86, -90}, {-86, -90}}, color = {0, 0, 255}));
  connect(capacityFadingCalc.Q_us, coulombSocCounter.Q_us) annotation(
    Line(points = {{-6, -58}, {-20, -58}, {-20, -74}, {70, -74}, {70, -18}, {40, -18}, {40, -18}}, color = {0, 0, 127}));
protected
  annotation(
    Diagram(coordinateSystem(initialScale = 0.1)),
    uses(Modelica(version = "3.2.3")),
    Documentation,
    experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-06, Interval = 100));
end TheveninBasedBattery;
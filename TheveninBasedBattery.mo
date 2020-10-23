model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
  Modelica.Electrical.Analog.Basic.Resistor R_0(R = 1) annotation(
    Placement(visible = true, transformation(extent = {{-58, 30}, {-38, 50}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor R_ts(R = 50) annotation(
    Placement(visible = true, transformation(extent = {{-20, 30}, {0, 50}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor R_tl(R = 50) annotation(
    Placement(visible = true, transformation(extent = {{20, 30}, {40, 50}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor C_ts(C = 500) annotation(
    Placement(visible = true, transformation(origin = {-10, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor C_tl(C = 5000) annotation(
    Placement(visible = true, transformation(origin = {30, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground GND annotation(
    Placement(visible = true, transformation(origin = {-78, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable Modelica.Electrical.Analog.Interfaces.VoltageSource sou(v = 2.7) annotation(
    Placement(visible = true, transformation(origin = {-78, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor U_ts annotation(
    Placement(visible = true, transformation(origin = {-10, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor U_tl annotation(
    Placement(visible = true, transformation(origin = {30, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(R_ts.p, R_0.n) annotation(
    Line(points = {{-20, 40}, {-38, 40}, {-38, 40}, {-38, 40}}, color = {0, 0, 255}));
  connect(C_ts.p, R_ts.p) annotation(
    Line(points = {{-20, 12}, {-20, 12}, {-20, 40}, {-20, 40}}, color = {0, 0, 255}));
  connect(C_ts.n, R_ts.n) annotation(
    Line(points = {{0, 12}, {0, 12}, {0, 40}, {0, 40}}, color = {0, 0, 255}));
  connect(R_tl.p, R_ts.n) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, 40}, {0, 40}}, color = {0, 0, 255}));
  connect(C_tl.p, R_tl.p) annotation(
    Line(points = {{20, 12}, {20, 12}, {20, 40}, {20, 40}}, color = {0, 0, 255}));
  connect(C_tl.n, R_tl.n) annotation(
    Line(points = {{40, 12}, {40, 40}}, color = {0, 0, 255}));
  connect(sou.n, GND.p) annotation(
    Line(points = {{-78, -20}, {-78, -20}, {-78, -40}, {-78, -40}}, color = {0, 0, 255}));
  connect(R_tl.n, GND.p) annotation(
    Line(points = {{40, 40}, {49, 40}, {49, -40}, {-78, -40}}, color = {0, 0, 255}));
  connect(R_0.p, sou.p) annotation(
    Line(points = {{-58, 40}, {-78, 40}, {-78, 0}}, color = {0, 0, 255}));
  connect(U_ts.n, R_ts.n) annotation(
    Line(points = {{0, 64}, {0, 64}, {0, 40}, {0, 40}}, color = {0, 0, 255}));
  connect(U_ts.p, R_ts.p) annotation(
    Line(points = {{-20, 64}, {-20, 64}, {-20, 40}, {-20, 40}}, color = {0, 0, 255}));
  connect(U_tl.p, R_tl.p) annotation(
    Line(points = {{20, 64}, {20, 64}, {20, 40}, {20, 40}}, color = {0, 0, 255}));
  connect(U_tl.n, R_tl.n) annotation(
    Line(points = {{40, 64}, {40, 64}, {40, 40}, {40, 40}}, color = {0, 0, 255}));
  annotation(
    Diagram(coordinateSystem(initialScale = 0.1)),
    uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")));
end TheveninBasedBattery;
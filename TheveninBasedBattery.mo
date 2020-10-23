model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
  constant Real C_bat = 3060;
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
  VariableVoltageSource sou annotation(
    Placement(visible = true, transformation(origin = {-92, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor U_ts annotation(
    Placement(visible = true, transformation(origin = {-10, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor U_tl annotation(
    Placement(visible = true, transformation(origin = {30, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.CurrentSensor I_bat annotation(
    Placement(visible = true, transformation(origin = {62, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1 / C_bat)  annotation(
    Placement(visible = true, transformation(origin = {18, -52}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1D SOC_to_U_bat( fileName="/home/midren/bachelor/soc_to_u_bat_tookup.txt", tableName = "tab",tableOnFile=true) annotation(
    Placement(visible = true, transformation(origin = {-76, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2)  annotation(
    Placement(visible = true, transformation(origin = {-22, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant SOC_init(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {18, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground GND annotation(
    Placement(visible = true, transformation(origin = {-92, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
  connect(R_0.p, sou.p) annotation(
    Line(points = {{-58, 40}, {-92, 40}, {-92, -26}}, color = {0, 0, 255}));
  connect(U_ts.n, R_ts.n) annotation(
    Line(points = {{0, 64}, {0, 64}, {0, 40}, {0, 40}}, color = {0, 0, 255}));
  connect(U_ts.p, R_ts.p) annotation(
    Line(points = {{-20, 64}, {-20, 64}, {-20, 40}, {-20, 40}}, color = {0, 0, 255}));
  connect(U_tl.p, R_tl.p) annotation(
    Line(points = {{20, 64}, {20, 64}, {20, 40}, {20, 40}}, color = {0, 0, 255}));
  connect(U_tl.n, R_tl.n) annotation(
    Line(points = {{40, 64}, {40, 64}, {40, 40}, {40, 40}}, color = {0, 0, 255}));
  connect(I_bat.i, integrator.u) annotation(
    Line(points = {{62, 29}, {62, -58.5}, {30, -58.5}, {30, -52}}, color = {0, 0, 127}));
  connect(SOC_init.y, Sum.u[1]) annotation(
    Line(points = {{7, -20}, {-6, -20}, {-6, -36}, {-10, -36}}, color = {0, 0, 127}));
  connect(integrator.y, Sum.u[2]) annotation(
    Line(points = {{7, -52}, {-6, -52}, {-6, -36}, {-10, -36}}, color = {0, 0, 127}));
  connect(SOC_to_U_bat.u[1], Sum.y) annotation(
    Line(points = {{-64, -36}, {-33, -36}}, color = {0, 0, 127}));
  connect(SOC_to_U_bat.y[1], sou.voltage) annotation(
    Line);
  connect(I_bat.p, R_tl.n) annotation(
    Line(points = {{52, 40}, {40, 40}}, color = {0, 0, 255}));
  connect(sou.n, GND.p) annotation(
    Line(points = {{-92, -46}, {-92, -76}}, color = {0, 0, 255}));
  connect(I_bat.n, GND.p) annotation(
    Line(points = {{72, 40}, {86, 40}, {86, -76}, {-92, -76}, {-92, -76}}, color = {0, 0, 255}));
  annotation(
    Diagram(coordinateSystem(initialScale = 0.1)),
    uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")),
  Documentation);
end TheveninBasedBattery;
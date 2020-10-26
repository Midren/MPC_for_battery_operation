model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
  constant Real C_bat = 3060;
  Modelica.Electrical.Analog.Basic.Resistor R_0(R = 1) annotation(
    Placement(visible = true, transformation(origin = {-48, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor R_tl(R = 50) annotation(
    Placement(visible = true, transformation(origin = {30, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor C_tl(C = 5000) annotation(
    Placement(visible = true, transformation(origin = {30, 12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  VariableVoltageSource sou annotation(
    Placement(visible = true, transformation(origin = {-88, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Sensors.CurrentSensor I_bat annotation(
    Placement(visible = true, transformation(origin = {62, 40}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
  Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1 / C_bat)  annotation(
    Placement(visible = true, transformation(origin = {16, -22}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1D SOC_to_U_bat( fileName="/home/midren/bachelor/soc_to_u_bat_tookup.txt", tableName = "tab",tableOnFile=true) annotation(
    Placement(visible = true, transformation(origin = {-72, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2)  annotation(
    Placement(visible = true, transformation(origin = {-26, -36}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant SOC_init(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {15, -43}, extent = {{7, -7}, {-7, 7}}, rotation = 0)));
  Load load(I_req = 2, v(start = 0.0001))  annotation(
    Placement(visible = true, transformation(origin = {42, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor R_ts(R = 50) annotation(
    Placement(visible = true, transformation(origin = {-10, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor C_ts(C = 500) annotation(
    Placement(visible = true, transformation(origin = {-10, 12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground GND annotation(
    Placement(visible = true, transformation(origin = {-88, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(I_bat.i, integrator.u) annotation(
    Line(points = {{62, 29}, {62, -20.5}, {23, -20.5}, {23, -22}}, color = {0, 0, 127}));
  connect(SOC_init.y, Sum.u[1]) annotation(
    Line(points = {{7, -43}, {-2, -43}, {-2, -36}, {-19, -36}}, color = {0, 0, 127}));
  connect(integrator.y, Sum.u[2]) annotation(
    Line(points = {{9, -22}, {-1.5, -22}, {-1.5, -36}, {-19, -36}}, color = {0, 0, 127}));
  connect(SOC_to_U_bat.u[1], Sum.y) annotation(
    Line(points = {{-60, -36}, {-33, -36}}, color = {0, 0, 127}));
  connect(SOC_to_U_bat.y[1], sou.voltage) annotation(
    Line);
  connect(R_ts.n, R_0.p) annotation(
    Line(points = {{-20, 40}, {-38, 40}, {-38, 40}, {-38, 40}}, color = {0, 0, 255}));
  connect(R_ts.p, R_tl.n) annotation(
    Line(points = {{0, 40}, {20, 40}, {20, 40}, {20, 40}}, color = {0, 0, 255}));
  connect(sou.p, GND.p) annotation(
    Line(points = {{-88, -46}, {-88, -46}, {-88, -80}, {-88, -80}}, color = {0, 0, 255}));
  connect(sou.n, R_0.n) annotation(
    Line(points = {{-88, -26}, {-88, 40}, {-58, 40}}, color = {0, 0, 255}));
  connect(C_tl.p, R_tl.p) annotation(
    Line(points = {{40, 12}, {40, 12}, {40, 40}, {40, 40}}, color = {0, 0, 255}));
  connect(R_tl.n, C_tl.n) annotation(
    Line(points = {{20, 40}, {20, 40}, {20, 12}, {20, 12}}, color = {0, 0, 255}));
  connect(R_ts.n, C_ts.n) annotation(
    Line(points = {{-20, 40}, {-20, 40}, {-20, 12}, {-20, 12}}, color = {0, 0, 255}));
  connect(R_ts.p, C_ts.p) annotation(
    Line(points = {{0, 40}, {0, 40}, {0, 12}, {0, 12}}, color = {0, 0, 255}));
  connect(load.p, GND.p) annotation(
    Line(points = {{32, -80}, {-88, -80}}, color = {0, 0, 255}));
  connect(I_bat.n, R_tl.p) annotation(
    Line(points = {{52, 40}, {40, 40}, {40, 40}, {40, 40}}, color = {0, 0, 255}));
  connect(I_bat.p, load.n) annotation(
    Line(points = {{72, 40}, {72, -80}, {52, -80}}, color = {0, 0, 255}));
protected
  annotation(
    Diagram(coordinateSystem(initialScale = 0.1)),
    uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")),
  Documentation);
end TheveninBasedBattery;
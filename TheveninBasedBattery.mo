model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
  constant Real C_bat = 3060;
  Modelica.Electrical.Analog.Basic.Resistor R_0(R = 1) annotation(
    Placement(visible = true, transformation(origin = {-48, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor R_tl(R = 50) annotation(
    Placement(visible = true, transformation(origin = {30, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor C_tl(C = 5000) annotation(
    Placement(visible = true, transformation(origin = {30, 12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.CurrentSensor I_bat annotation(
    Placement(visible = true, transformation(origin = {60, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1 / C_bat) annotation(
    Placement(visible = true, transformation(origin = {14, -22}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1D SOC_to_U_bat(fileName = "/home/midren/bachelor/soc_to_u_bat_tookup.txt", tableName = "tab", tableOnFile = true) annotation(
    Placement(visible = true, transformation(origin = {-44, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2) annotation(
    Placement(visible = true, transformation(origin = {-18, -36}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant SOC_init(k = 1) annotation(
    Placement(visible = true, transformation(origin = {15, -43}, extent = {{7, -7}, {-7, 7}}, rotation = 0)));
  Load load(I_req = 0.08, v(start = 0.0001)) annotation(
    Placement(visible = true, transformation(origin = {42, -80}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
  Modelica.Electrical.Analog.Basic.Resistor R_ts(R = 50) annotation(
    Placement(visible = true, transformation(origin = {-10, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor C_ts(C = 500) annotation(
    Placement(visible = true, transformation(origin = {-10, 12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground GND annotation(
    Placement(visible = true, transformation(origin = {-88, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VariableVoltageSource signalVoltage(v(start = 4.1))  annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-100, -36})));
  Modelica.Electrical.Analog.Sensors.VoltageSensor U_ocv annotation(
    Placement(visible = true, transformation(origin = {-74, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum voltageChange(k = {1, -1}, nin = 2)  annotation(
    Placement(visible = true, transformation(origin = {-76, -36}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
equation
  // main circuit
  connect(signalVoltage.p, R_0.n) annotation(
    Line(points = {{-100, -26}, {-100, 40}, {-58, 40}}, color = {0, 0, 255}));
  
  connect(R_0.p, R_ts.n) annotation(
    Line(points = {{-20, 40}, {-38, 40}, {-38, 40}, {-38, 40}}, color = {0, 0, 255}));
  connect(R_ts.n, C_ts.n) annotation(
    Line(points = {{-20, 40}, {-20, 40}, {-20, 12}, {-20, 12}}, color = {0, 0, 255}));
  connect(R_ts.p, C_ts.p) annotation(
    Line(points = {{0, 40}, {0, 40}, {0, 12}, {0, 12}}, color = {0, 0, 255}));
    
  connect(R_ts.p, R_tl.n) annotation(
    Line(points = {{0, 40}, {20, 40}, {20, 40}, {20, 40}}, color = {0, 0, 255}));
  connect(R_tl.n, C_tl.n) annotation(
    Line(points = {{20, 40}, {20, 40}, {20, 12}, {20, 12}}, color = {0, 0, 255}));
  connect(R_tl.p, C_tl.p) annotation(
    Line(points = {{40, 12}, {40, 12}, {40, 40}, {40, 40}}, color = {0, 0, 255}));
  
  connect(R_tl.p, I_bat.p) annotation(
    Line(points = {{40, 40}, {50, 40}}, color = {0, 0, 255}));
  connect(GND.p, signalVoltage.n) annotation(
    Line(points = {{-100, -46}, {-100, -80}, {-88, -80}}, color = {0, 0, 255}));
    
  // Find required voltage
  connect(I_bat.i, integrator.u) annotation(
    Line(points = {{60, 29}, {60, -20.5}, {21, -20.5}, {21, -22}}, color = {0, 0, 127}));
  connect(integrator.y, Sum.u[2]) annotation(
    Line(points = {{7, -22}, {-1.5, -22}, {-1.5, -36}, {-11, -36}}, color = {0, 0, 127}));  
  connect(SOC_init.y, Sum.u[1]) annotation(
    Line(points = {{7.3, -43}, {-2, -43}, {-2, -36}, {-11, -36}}, color = {0, 0, 127}));
  connect(Sum.y, SOC_to_U_bat.u[1]) annotation(
    Line(points = {{-32, -36}, {-25, -36}}, color = {0, 0, 127}));

  // Find voltage difference and modify signalVoltage
  connect(signalVoltage.p, U_ocv.p) annotation(
    Line(points = {{-100, -26}, {-100, -19}, {-84, -19}, {-84, -18}}, color = {0, 0, 255}));
  connect(signalVoltage.n, U_ocv.n) annotation(
    Line(points = {{-64, -18}, {-57, -18}, {-57, -46}, {-100, -46}}, color = {0, 0, 255}));
  connect(U_ocv.v, voltageChange.u[2]) annotation(
    Line(points = {{-74, -29}, {-74, -36}, {-69, -36}}, color = {0, 0, 127}));
  connect(SOC_to_U_bat.y[2], voltageChange.u[1]) annotation(
    Line(points = {{-54, -36}, {-69, -36}}, color = {0, 0, 127}, thickness = 0.5));
  connect(voltageChange.y, signalVoltage.voltage) annotation(
    Line(points = {{-83, -36}, {-88, -36}}, color = {0, 0, 127}));
  connect(GND.p, load.n) annotation(
    Line(points = {{-88, -80}, {32, -80}}, color = {0, 0, 255}));
  connect(load.p, I_bat.n) annotation(
    Line(points = {{52, -80}, {70, -80}, {70, 40}}, color = {0, 0, 255}));
  annotation(
    Diagram(coordinateSystem(initialScale = 0.1)),
    uses(Modelica(version = "3.2.3")),
    Documentation,
    experiment(StartTime = 0, StopTime = 3600, Tolerance = 1e-06, Interval = 1.00056));
end TheveninBasedBattery;
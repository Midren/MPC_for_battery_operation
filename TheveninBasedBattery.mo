model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
  constant Real C_bat = 3060;
  Real SoC;
  ChargeDependentResistor R_s(R_0 = 0.07446, k1=0.1562, k2=-24.37) annotation(
    Placement(visible = true, transformation(origin = {-48, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ChargeDependentResistor R_ts(R_0 = 0.04669, k1=0.3208, k2=-29.14) annotation(
    Placement(visible = true, transformation(origin = {-10, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ChargeDependentCapacitor C_ts(C_0 = 703.6, k1=-752.9, k2=-13.51) annotation(
    Placement(visible = true, transformation(origin = {-10, 12}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  ChargeDependentResistor R_tl(R_0 = 0.04669, k1=6.603, k2=-29.14) annotation(
    Placement(visible = true, transformation(origin = {30, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ChargeDependentCapacitor C_tl(C_0 = 4475, k1=-6056, k2=-27.12) annotation(
    Placement(visible = true, transformation(origin = {30, 12}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
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
  connect(signalVoltage.p, R_s.n) annotation(
    Line(points = {{-100, -26}, {-100, 40}, {-58, 40}}, color = {0, 0, 255}));
  
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
  
  SoC = Sum.y;
  
  connect(Sum.y, R_s.SoC) annotation(
    Line(points = {{-24, -36}, {-28, -36}, {-28, 0}, {-74, 0}, {-74, 60}, {-48, 60}, {-48, 52}, {-48, 52}, {-48, 52}}, color = {0, 0, 127}));
  connect(Sum.y, R_ts.SoC) annotation(
    Line(points = {{-24, -36}, {-28, -36}, {-28, 0}, {-74, 0}, {-74, 60}, {-10, 60}, {-10, 52}, {-10, 52}}, color = {0, 0, 127}));
  connect(Sum.y, R_tl.SoC) annotation(
    Line(points = {{-24, -36}, {-28, -36}, {-28, 0}, {-74, 0}, {-74, 60}, {30, 60}, {30, 52}, {30, 52}}, color = {0, 0, 127}));
  connect(Sum.y, C_ts.SoC) annotation(
    Line(points = {{-24, -36}, {-28, -36}, {-28, -6}, {-10, -6}, {-10, 0}, {-10, 0}}, color = {0, 0, 127}));
  connect(Sum.y, C_tl.SoC) annotation(
    Line(points = {{-24, -36}, {-28, -36}, {-28, -6}, {30, -6}, {30, 0}, {30, 0}}, color = {0, 0, 127}));
  
  when SoC < 0 then
    terminate("battery has discharged");
  end when;
  
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
    experiment(StartTime = 0, StopTime = 36000, Tolerance = 1e-06, Interval = 0.100083));
end TheveninBasedBattery;
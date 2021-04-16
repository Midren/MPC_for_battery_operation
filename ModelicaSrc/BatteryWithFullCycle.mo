package BatteryMPC
  model BatteryWithFullCycle
    model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
      extends Modelica.Electrical.Analog.Interfaces.TwoPin;

      model VariableSource "Generate signal of type Real"
        Real height "Height of step" annotation(
          Dialog(groupImage = "modelica://Modelica/Resources/Images/Blocks/Sources/Step.png"));
        extends Modelica.Blocks.Interfaces.SignalSource;
      equation
        y = offset + (if time < startTime then 0 else height);
      end VariableSource;

      model VariableVoltageSource
        extends Modelica.Electrical.Analog.Interfaces.VoltageSource(redeclare VariableSource signalSource);
        Modelica.Blocks.Interfaces.RealInput voltage "Connector of Real input signals" annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 270, origin = {0, 100})));
      equation
        signalSource.height = signalSource.height + voltage;
        annotation(
          uses(Modelica(version = "3.2.3")));
      end VariableVoltageSource;

      model voltageSourceSocDependant "Interface for voltage sources"
        extends Modelica.Electrical.Analog.Interfaces.TwoPin;
        parameter String SocToULookupTableFileName;
        Modelica.Electrical.Analog.Sensors.VoltageSensor U_ocv annotation(
          Placement(visible = true, transformation(origin = {-2, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Tables.CombiTable1D SOC_to_U_bat(fileName = SocToULookupTableFileName, tableName = "tab", tableOnFile = true) annotation(
          Placement(visible = true, transformation(origin = {-12, 74}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Sum voltageChange(k = {1, -1}, nin = 2) annotation(
          Placement(visible = true, transformation(origin = {-62, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
        VariableVoltageSource signalVoltage(v(start = 4.1)) annotation(
          Placement(visible = true, transformation(origin = {-2, -46}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput SoC annotation(
          Placement(visible = true, transformation(origin = {0, 122}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 122}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      equation
        connect(U_ocv.v, voltageChange.u[1]) annotation(
          Line(points = {{-2, 45}, {-48, 45}, {-48, 60}, {-54, 60}}, color = {0, 0, 127}));
        connect(voltageChange.y, signalVoltage.voltage) annotation(
          Line(points = {{-69, 60}, {-88.25, 60}, {-88.25, 18}, {-2.5625, 18}, {-2.5625, -36}, {-2, -36}}, color = {0, 0, 127}));
        connect(SoC, SOC_to_U_bat.u[1]) annotation(
          Line(points = {{0, 122}, {0, 74}}, color = {0, 0, 127}));
        connect(voltageChange.u[2], SOC_to_U_bat.y[1]) annotation(
          Line(points = {{-54, 60}, {-49, 60}, {-49, 74}, {-23, 74}}, color = {0, 0, 127}, thickness = 0.5));
        connect(U_ocv.p, signalVoltage.p) annotation(
          Line(points = {{8, 34}, {8, -6}, {-12, -6}, {-12, -46}}, color = {0, 0, 255}));
        connect(U_ocv.n, signalVoltage.n) annotation(
          Line(points = {{-12, 34}, {-12, -6}, {8, -6}, {8, -46}}, color = {0, 0, 255}));
        connect(signalVoltage.p, p) annotation(
          Line(points = {{-12, -46}, {-60, -46}, {-60, 0}, {-100, 0}, {-100, 0}}, color = {0, 0, 255}));
        connect(signalVoltage.n, n) annotation(
          Line(points = {{8, -46}, {40, -46}, {40, 0}, {100, 0}, {100, 0}}, color = {0, 0, 255}));
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-50,50},{50,-50}},
                lineColor={0,0,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-150,60},{150,100}},
                textString="%name",
                lineColor={0,0,255}),
              Line(points={{-90,0},{90,0}}, color={0,0,255}),
              Line(points={{-80,20},{-60,20}}, color={0,0,255}),
              Line(points={{-70,30},{-70,10}}, color={0,0,255}),
              Line(points={{60,20},{80,20}}, color={0,0,255})}),
          Documentation(revisions="<html>
      <ul>
      <li><em> 1998   </em>
           by Christoph Clauss<br> initially implemented<br>
           </li>
      </ul>
      </html>", info="<html>
      <p>The VoltageSource partial model prepares voltage sources by providing the pins, and the offset and startTime parameters, which are the same at all voltage sources. The source behavior is taken from Modelica.Blocks signal sources by inheritance and usage of the replaceable possibilities.</p>
      </html>"));
      end voltageSourceSocDependant;

      model ChargeDependentResistor
        record Parameters
          Modelica.SIunits.Resistance R_0;
          Real k1;
          Real k2;
          Real k3;
        end Parameters;

        extends Modelica.Electrical.Analog.Basic.VariableResistor;
        Modelica.Blocks.Interfaces.RealInput SoC;
        input Parameters params;
      equation
        R = params.R_0 + params.k1 * exp(-params.k2 * SoC) + params.k3 * SoC;
      end ChargeDependentResistor;

      model ChargeDependentCapacitor
        extends Modelica.Electrical.Analog.Basic.VariableCapacitor;

        record Parameters
          Modelica.SIunits.Capacitance C_0;
          Real k1;
          Real k2;
          Real k3;
          Real k4;
          Real k5;
          Real k6;
        end Parameters;

        Modelica.Blocks.Interfaces.RealInput SoC;
        input Parameters params;
      equation
        C = max(params.C_0 + params.k1 * SoC + params.k2 * SoC ^ 2 + params.k3 * SoC ^ 3 + params.k4 * SoC ^ 4 + params.k5 * SoC ^ 5 + params.k6 * SoC ^ 6, 0);
      end ChargeDependentCapacitor;

      model CoulombSocCounter
        Modelica.Blocks.Interfaces.RealInput I_bat "Connector of Real input signals" annotation(
          Placement(visible = true, transformation(origin = {120, 26}, extent = {{-20, 20}, {20, -20}}, rotation = 180), iconTransformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
        Modelica.Blocks.Sources.Constant SOC_init(k = 1) annotation(
          Placement(visible = true, transformation(origin = {17, 49}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1) annotation(
          Placement(visible = true, transformation(origin = {59, 25}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
        Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2) annotation(
          Placement(visible = true, transformation(origin = {-26, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput SoC annotation(
          Placement(visible = true, transformation(origin = {-110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Nonlinear.Limiter socLimiter(limitsAtInit = true, strict = true, uMax = 1, uMin = 0.012) annotation(
          Placement(visible = true, transformation(origin = {-60, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Q_us annotation(
          Placement(visible = true, transformation(origin = {120, -26}, extent = {{20, -20}, {-20, 20}}, rotation = 0), iconTransformation(origin = {-40, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
        Modelica.Blocks.Math.Division division annotation(
          Placement(visible = true, transformation(origin = {14, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      equation
  connect(SOC_init.y, Sum.u[1]) annotation(
          Line(points = {{5, 49}, {-4, 49}, {-4, 0}, {-14, 0}}, color = {0, 0, 127}));
  connect(I_bat, integrator.u) annotation(
          Line(points = {{120, 26}, {82, 26}, {82, 25}, {72, 25}}, color = {0, 0, 127}));
  connect(Sum.y, socLimiter.u) annotation(
          Line(points = {{-37, 0}, {-48, 0}}, color = {0, 0, 127}));
  connect(socLimiter.y, SoC) annotation(
          Line(points = {{-71, 0}, {-110, 0}}, color = {0, 0, 127}));
  connect(Q_us, division.u2) annotation(
          Line(points = {{120, -26}, {26, -26}}, color = {0, 0, 127}));
  connect(integrator.y, division.u1) annotation(
          Line(points = {{47, 25}, {40, 25}, {40, -14}, {26, -14}}, color = {0, 0, 127}));
  connect(division.y, Sum.u[2]) annotation(
          Line(points = {{3, -20}, {-4.5, -20}, {-4.5, 0}, {-14, 0}}, color = {0, 0, 127}));
        annotation(
          uses(Modelica(version = "3.2.3")));
      end CoulombSocCounter;

      model CapacityFadingCalculator
        model IntegratorWithReset
          extends Modelica.Blocks.Continuous.Integrator;
          Modelica.Blocks.Interfaces.BooleanInput reset annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = -360)));
        equation
          when reset then
            reinit(y, y_start);
          end when;
        end IntegratorWithReset;

        record Parameters
          Real K_co = 3.66e-5;
          Real K_ex = 0.717;
          Real K_soc = 0.916;
        end Parameters;

        parameter Real C_bat "Nominal capacity in Ah";
        parameter Parameters params;
        Modelica.Blocks.Logical.LessThreshold isCharging(threshold = 0);
        Real cycleStartTime(start = 0);
        Real cycleLen;
        Real SoC_avg(start = 0);
        Real SoC_dev(start = 0);
        Real n_m(start = 0) "Effective number of throughput cycles";
        Real L(start = 0) "Life ageing parameter";
        Real stepCapacityFade(start = 0);
        Real capacityFade(start = 0);
        Modelica.Blocks.Interfaces.RealInput SoC annotation(
          Placement(visible = true, transformation(origin = {0, 118}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput I annotation(
          Placement(visible = true, transformation(origin = {120, 0}, extent = {{-20, 20}, {20, -20}}, rotation = 180), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
        BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CapacityFadingCalculator.IntegratorWithReset socIntegrator(y_start = 0) annotation(
          Placement(visible = true, transformation(origin = {40, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product socSquare annotation(
          Placement(visible = true, transformation(origin = {-34, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CapacityFadingCalculator.IntegratorWithReset socSquareIntegrator(y_start = 0) annotation(
          Placement(visible = true, transformation(origin = {-78, 54}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CapacityFadingCalculator.IntegratorWithReset Ah_throughput(y_start = 0) annotation(
          Placement(visible = true, transformation(origin = {6, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Abs I_abs annotation(
          Placement(visible = true, transformation(origin = {42, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Q_us(start = C_bat) annotation(
          Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180), iconTransformation(origin = {0, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
      equation
  connect(isCharging.u, I) annotation(
          Line);
  connect(socIntegrator.reset, isCharging.y) annotation(
          Line);
  connect(socSquareIntegrator.reset, isCharging.y) annotation(
          Line);
  connect(Ah_throughput.reset, isCharging.y) annotation(
          Line);
        L = 1 - Q_us / C_bat;
        when isCharging.y == false then
          cycleLen = time - pre(cycleStartTime);
          SoC_avg = 1 / cycleLen * pre(socIntegrator.y);
          SoC_dev = 2 * sqrt(3 / cycleLen) * sqrt(pre(socSquareIntegrator.y) - 2 * SoC_avg * pre(socIntegrator.y) + cycleLen * SoC_avg * SoC_avg);
          n_m = pre(Ah_throughput.y) / (2 * C_bat);
          stepCapacityFade = params.K_co * n_m * exp((SoC_dev - 1) / params.K_ex) * exp(params.K_soc * ((SoC_avg - 0.5) / 0.25)) * (1 - pre(L));
          Q_us = pre(Q_us) - stepCapacityFade;
          capacityFade = pre(capacityFade) + stepCapacityFade;
          cycleStartTime = time;
        elsewhen isCharging.y == true then
          cycleLen = pre(cycleLen);
          SoC_avg = pre(SoC_avg);
          SoC_dev = pre(SoC_dev);
          n_m = pre(n_m);
          stepCapacityFade = pre(stepCapacityFade);
          capacityFade = pre(capacityFade);
          Q_us = pre(Q_us);
          cycleStartTime = pre(cycleStartTime);
        end when;
  connect(SoC, socIntegrator.u) annotation(
          Line(points = {{0, 118}, {0, 54}, {28, 54}}, color = {0, 0, 127}));
  connect(SoC, socSquare.u1) annotation(
          Line(points = {{0, 118}, {-1, 118}, {-1, 100}, {0, 100}, {0, 49}, {-22, 49}, {-22, 48}}, color = {0, 0, 127}));
  connect(SoC, socSquare.u2) annotation(
          Line(points = {{0, 118}, {0, 60}, {-22, 60}}, color = {0, 0, 127}));
  connect(socSquare.y, socSquareIntegrator.u) annotation(
          Line(points = {{-45, 54}, {-66, 54}}, color = {0, 0, 127}));
  connect(I, I_abs.u) annotation(
          Line(points = {{120, 0}, {54, 0}}, color = {0, 0, 127}));
  connect(I_abs.y, Ah_throughput.u) annotation(
          Line(points = {{31, 0}, {18, 0}}, color = {0, 0, 127}));
        annotation(
          uses(Modelica(version = "3.2.3")),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
      end CapacityFadingCalculator;

      model SeriesResistor
        record Parameters
          Modelica.SIunits.Resistance R_0;
          Real k1;
          Real k2;
          Real k3;
          Real k4;
        end Parameters;

        extends Modelica.Electrical.Analog.Basic.VariableResistor;
        Modelica.Blocks.Interfaces.RealInput SoC;
        input Parameters params;
      equation
        R = params.R_0 + params.k1 * SoC + params.k2 * SoC ^ 2 + params.k3 * SoC ^ 3 + params.k4 * SoC ^ 4;
      end SeriesResistor;

      model ShortTimeTransientResistor
        record Parameters
          Modelica.SIunits.Resistance R_0;
          Real k1;
          Real k2;
          Real k3;
        end Parameters;

        extends Modelica.Electrical.Analog.Basic.VariableResistor;
        Modelica.Blocks.Interfaces.RealInput SoC;
        input Parameters params;
      equation
        R = params.R_0 + params.k1 * exp(params.k2 * SoC) + params.k3 * SoC;
      end ShortTimeTransientResistor;

      model LongTimeTransientResistor
        record Parameters
          Modelica.SIunits.Resistance R_0;
          Real k1;
          Real k2;
          Real k3;
        end Parameters;

        extends Modelica.Electrical.Analog.Basic.VariableResistor;
        Modelica.Blocks.Interfaces.RealInput SoC;
        input Parameters params;
      equation
        R = params.R_0 + params.k1 * exp(params.k2 * SoC) + params.k3 * SoC;
      end LongTimeTransientResistor;

      record TheveninBasedModelParameters
        SeriesResistor.Parameters R_s;
        ChargeDependentResistor.Parameters R_ts;
        ChargeDependentResistor.Parameters R_tl;
        ChargeDependentCapacitor.Parameters C_ts;
        ChargeDependentCapacitor.Parameters C_tl;
      end TheveninBasedModelParameters;

      constant TheveninBasedModelParameters dischargingParams(R_s = SeriesResistor.Parameters(R_0 = 8.98e-2, k1 = -7.216e-2, k2 = 2.273e-1, k3 = -2.892e-1, k4 = 1.298e-1), R_ts = ChargeDependentResistor.Parameters(R_0 = 1.827e-2, k1 = 1.080e-2, k2 = 11.03, k3 = -6.463e-3), R_tl = ChargeDependentResistor.Parameters(R_0 = 4.722e-2, k1 = 2.95e-1, k2 = 20.00, k3 = -2.420e-2), C_ts = ChargeDependentCapacitor.Parameters(C_0 = 389.7, k1 = 1408, k2 = -1007, k3 = 169.7, k4 = 0, k5 = 0, k6 = 0), C_tl = ChargeDependentCapacitor.Parameters(C_0 = 2.232e3, k1 = -3.102e4, k2 = 5.998e5, k3 = -2.958e6, k4 = 6.271e6, k5 = -6.007e6, k6 = 2.130e6));
      constant TheveninBasedModelParameters chargingParams(R_s = SeriesResistor.Parameters(R_0 = 8.210e-2, k1 = -4.1006e-2, k2 = 1.609e-1, k3 = -2.518e-1, k4 = 1.369e-1), R_ts = ChargeDependentResistor.Parameters(R_0 = 1.4e-2, k1 = 7.13e-11, k2 = -21.11, k3 = 0), R_tl = ChargeDependentResistor.Parameters(R_0 = 3.1e-2, k1 = 8.913e-15, k2 = -32.23, k3 = 4.473e-3), C_ts = ChargeDependentCapacitor.Parameters(C_0 = 6.849e2, k1 = 2.340e3, k2 = -1.013e4, k3 = 1.723e4, k4 = -1.026e4, k5 = 0, k6 = 0), C_tl = ChargeDependentCapacitor.Parameters(C_0 = 7.144e3, k1 = 2.283e4, k2 = -8.124e4, k3 = -4.009e3, k4 = 2.042e5, k5 = -1.541e5, k6 = 0));
      parameter Real C_bat = 1.1;
      parameter String SocToOcvTableFileName = "/home/developer/modelica/soc_to_u_bat_tookup.txt";
      parameter CapacityFadingCalculator.Parameters capacityFadingParams(K_co = 3.66e-5, K_ex = 0.717, K_soc = 0.916);
      Real SoH(start = 1);
      TheveninBasedModelParameters currentParams;
      SeriesResistor R_s annotation(
        Placement(visible = true, transformation(origin = {-48, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      ChargeDependentResistor R_ts annotation(
        Placement(visible = true, transformation(origin = {-10, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      ChargeDependentCapacitor C_ts annotation(
        Placement(visible = true, transformation(origin = {-10, 12}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
      ChargeDependentResistor R_tl annotation(
        Placement(visible = true, transformation(origin = {30, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      ChargeDependentCapacitor C_tl annotation(
        Placement(visible = true, transformation(origin = {30, 12}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor I_bat annotation(
        Placement(visible = true, transformation(origin = {60, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
      BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CoulombSocCounter coulombSocCounter annotation(
        Placement(visible = true, transformation(origin = {34, -58}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.voltageSourceSocDependant voltageSource(SocToULookupTableFileName = SocToOcvTableFileName) annotation(
        Placement(visible = true, transformation(origin = {-76, 0}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
      BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CapacityFadingCalculator capacityFadingCalc(C_bat = C_bat, params = capacityFadingParams) annotation(
        Placement(visible = true, transformation(origin = {-10, -86}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
      SoH = 1 - capacityFadingCalc.capacityFade / (0.2 * C_bat);
      R_s.params = currentParams.R_s;
      R_ts.params = currentParams.R_ts;
      C_ts.params = currentParams.C_ts;
      R_tl.params = currentParams.R_tl;
      C_tl.params = currentParams.C_tl;
      if noEvent(I_bat.i < 0) then
        currentParams = chargingParams;
      else
        currentParams = dischargingParams;
      end if;
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
      connect(I_bat.i, coulombSocCounter.I_bat) annotation(
        Line(points = {{60, 30}, {60, -64}, {44, -64}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, C_ts.SoC) annotation(
        Line(points = {{23, -58}, {-10, -58}, {-10, 0}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, C_tl.SoC) annotation(
        Line(points = {{23, -58}, {-10, -58}, {-10, -8}, {30, -8}, {30, 0}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, R_tl.SoC) annotation(
        Line(points = {{23, -58}, {6, -58}, {6, 60}, {30, 60}, {30, 52}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, R_ts.SoC) annotation(
        Line(points = {{23, -58}, {6, -58}, {6, 60}, {-10, 60}, {-10, 52}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, R_s.SoC) annotation(
        Line(points = {{23, -58}, {6, -58}, {6, 60}, {-48, 60}, {-48, 52}}, color = {0, 0, 127}));
  connect(voltageSource.p, R_s.n) annotation(
        Line(points = {{-66, 0}, {-66, 40}, {-58, 40}}, color = {0, 0, 255}));
  connect(coulombSocCounter.SoC, voltageSource.SoC) annotation(
        Line(points = {{23, -58}, {-75.75, -58}, {-75.75, -12}, {-76, -12}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, capacityFadingCalc.SoC) annotation(
        Line(points = {{23, -58}, {-10, -58}, {-10, -76}}, color = {0, 0, 127}));
  connect(capacityFadingCalc.I, I_bat.i) annotation(
        Line(points = {{0, -86}, {60, -86}, {60, 30}}, color = {0, 0, 127}));
  connect(capacityFadingCalc.Q_us, coulombSocCounter.Q_us) annotation(
        Line(points = {{-21, -86}, {-36, -86}, {-36, -36}, {53, -36}, {53, -54}, {44, -54}}, color = {0, 0, 127}));
  connect(p, voltageSource.n) annotation(
        Line(points = {{-100, 0}, {-86, 0}, {-86, 0}}, color = {0, 0, 255}));
      connect(I_bat.n, n) annotation(
        Line(points = {{70, 40}, {80, 40}, {80, 0}, {100, 0}}, color = {0, 0, 255}));
  connect(C_tl.p, R_tl.p) annotation(
        Line(points = {{40, 12}, {40, 40}}, color = {0, 0, 255}));
    protected
      annotation(
        Diagram(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {0.599966, 23.2454}, extent = {{140.429, -43.2454}, {-140.429, 43.2454}}), Text(origin = {78, 63}, extent = {{-28, 5}, {28, -5}}, textString = "Thevenin-based Battery")}),
        uses(Modelica(version = "3.2.3")),
        Documentation,
        experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-06, Interval = 100));
    end TheveninBasedBattery;

    parameter Real C_bat = 1.1;
    parameter Real I_dis = C_bat/3600;
    BatteryWithFullCycle.TheveninBasedBattery theveninBasedBattery(C_bat=C_bat) annotation(
      Placement(visible = true, transformation(origin = {-4, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-50, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalCurrent pulseLoad annotation(
      Placement(visible = true, transformation(origin = {0, -16}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I_req annotation(
      Placement(visible = true, transformation(origin = {-100, -68}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    connect(ground.p, theveninBasedBattery.p) annotation(
      Line(points = {{-50, -16}, {-50, 20}, {-14, 20}}, color = {0, 0, 255}));
    connect(pulseLoad.n, ground.p) annotation(
      Line(points = {{-10, -16}, {-50, -16}}, color = {0, 0, 255}));
    connect(theveninBasedBattery.n, pulseLoad.p) annotation(
      Line(points = {{6, 20}, {20, 20}, {20, -16}, {10, -16}}, color = {0, 0, 255}));
    connect(I_req, pulseLoad.i) annotation(
      Line(points = {{-100, -68}, {0, -68}, {0, -28}}, color = {0, 0, 127}));
    annotation(
      uses(Modelica(version = "3.2.3")),
      experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-6, Interval = 100));
  end BatteryWithFullCycle;

end BatteryMPC;
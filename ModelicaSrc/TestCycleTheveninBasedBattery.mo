model TestCycleTheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
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
    annotation(
      uses(Modelica(version = "3.2.3")));
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
    R = 3600*(params.R_0 + params.k1 * exp(-params.k2 * SoC) + params.k3 * SoC);
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
    C = max(params.C_0 + params.k1 * SoC + params.k2 * SoC ^ 2 + params.k3 * SoC ^ 3 + params.k4 * SoC ^ 4 + params.k5 * SoC ^ 5 + params.k6 * SoC ^ 6, 0)/3600;
  end ChargeDependentCapacitor;

  model CoulombSocCounter
    Modelica.Blocks.Interfaces.RealInput I "Connector of Real input signals" annotation(
      Placement(visible = true, transformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 270), iconTransformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
    Modelica.Blocks.Sources.Constant SOC_init(k = 1) annotation(
      Placement(visible = true, transformation(origin = {63, -21}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
    Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1) annotation(
      Placement(visible = true, transformation(origin = {63, 23}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
    Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2) annotation(
      Placement(visible = true, transformation(origin = {-26, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput SoC annotation(
      Placement(visible = true, transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Nonlinear.Limiter socLimiter(limitsAtInit = true, strict = true, uMax = 1, uMin = 0.012) annotation(
      Placement(visible = true, transformation(origin = {-60, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput Q_us annotation(
      Placement(visible = true, transformation(origin = {-40, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-40, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Math.Division division annotation(
      Placement(visible = true, transformation(origin = {16, 18}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(SOC_init.y, Sum.u[1]) annotation(
      Line(points = {{51, -21}, {10, -21}, {10, -10}, {-14, -10}}, color = {0, 0, 127}));
    connect(I, integrator.u) annotation(
      Line(points = {{60, 100}, {59, 100}, {59, 66}, {82, 66}, {82, 23}, {76, 23}}, color = {0, 0, 127}));
    connect(Sum.y, socLimiter.u) annotation(
      Line(points = {{-37, -10}, {-48, -10}}, color = {0, 0, 127}));
    connect(socLimiter.y, SoC) annotation(
      Line(points = {{-71, -10}, {-76, -10}, {-76, -90}, {0, -90}, {0, -110}}, color = {0, 0, 127}));
    connect(Q_us, division.u2) annotation(
      Line(points = {{-40, 100}, {-40, 62}, {36, 62}, {36, 12}, {28, 12}}, color = {0, 0, 127}));
    connect(integrator.y, division.u1) annotation(
      Line(points = {{51, 23}, {40, 23}, {40, 24}, {28, 24}}, color = {0, 0, 127}));
    connect(division.y, Sum.u[2]) annotation(
      Line(points = {{5, 18}, {-4, 18}, {-4, -10}, {-14, -10}}, color = {0, 0, 127}));
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
    
    block VarianceWithReset
      extends Modelica.Blocks.Icons.Block;
      parameter Modelica.SIunits.Time t_eps(min=100*Modelica.Constants.eps)=1e-7
        "Variance calculation starts at startTime + t_eps"
        annotation(Dialog(group="Advanced"));
    
      Modelica.Blocks.Interfaces.RealInput u "Noisy input signal" annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput y "Variance of the input signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.BooleanInput reset annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = -360)));
    protected  
      Real mu "Mean value (state variable)";
      Real var "Variance (state variable)";
      Real t_0(fixed=false, start=0) "Start time";
    initial equation
      t_0 = time;
      mu  = u;
      var = 0;
    equation
      when reset then
        reinit(mu, u);
        reinit(var, 0);
        t_0 = time;
      end when;
      //t_0 = t_0 + 0;
      der(mu)  = noEvent(if time >= t_0 + t_eps then (u-mu)/(time-t_0)             else 0);
      der(var) = noEvent(if time >= t_0 + t_eps then ((u-mu)^2 - var)/(time - t_0) else 0);
      y        = noEvent(if time >= t_0 + t_eps then max(var,0)                    else 0);
    end VarianceWithReset;
  
    block MeanWithReset
        "Calculates the empirical expectation (mean) value of its input signal"
      extends Modelica.Blocks.Icons.Block;
      parameter Modelica.SIunits.Time t_eps(min= 100*Modelica.Constants.eps)=1e-7
        "Mean value calculation starts at startTime + t_eps"
        annotation(Dialog(group="Advanced"));
    
      Modelica.Blocks.Interfaces.RealInput u "Noisy input signal" annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        "Expectation (mean) value of the input signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.BooleanInput reset annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = -360)));
    protected
      Real mu "Internal integrator variable";
      Real t_0(fixed=false, start=0) "Start time";
    initial equation
      t_0 = time;
      mu  = u;
    equation
      when reset then
        reinit(mu, u);
        t_0 = time;
      end when;
      //t_0 = t_0 + 0;
      der(mu) = noEvent(if time >= t_0 + t_eps then (u-mu)/(time-t_0) else 0);
      y       = noEvent(if time >= t_0 + t_eps then mu                else u);
    end MeanWithReset;
  
    block StandardDeviationWithReset
      "Calculates the empirical standard deviation of its input signal"
      extends Modelica.Blocks.Icons.Block;
      parameter Modelica.SIunits.Time t_eps(min=100*Modelica.Constants.eps)=1e-7
        "Standard deviation calculation starts at startTime + t_eps"
        annotation(Dialog(group="Advanced"));
    
      Modelica.Blocks.Interfaces.RealInput u "Noisy input signal" annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        "Standard deviation of the input signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.BooleanInput reset annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = -360)));
    
      VarianceWithReset variance(t_eps=t_eps)
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Blocks.Math.Sqrt sqrt1
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
    equation
      connect(variance.reset, reset);
      connect(variance.u, u) annotation (Line(
          points={{-62,0},{-120,0}}, color={0,0,127}));
      connect(sqrt1.u, variance.y) annotation (Line(
          points={{-22,0},{-39,0}}, color={0,0,127}));
      connect(sqrt1.y, y) annotation (Line(
          points={{1,0},{110,0}}, color={0,0,127}));
      annotation (Documentation(revisions="<html>
    <table border=1 cellspacing=0 cellpadding=2>
    <tr><th>Date</th> <th align=\"left\">Description</th></tr>
    
    <tr><td> June 22, 2015 </td>
      <td>
    
    <table border=0>
    <tr><td>
           <img src=\"modelica://Modelica/Resources/Images/Blocks/Noise/dlr_logo.png\">
    </td><td valign=\"bottom\">
           Initial version implemented by
           A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
           <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
    </td></tr></table>
    </td></tr>
    
    </table>
    </html>",                                   info="<html>
    <p>This block calculates the standard deviation of its input signal. The standard deviation is the square root of the signal&#39;s variance:</p>
    <blockquote>
    <pre>y = sqrt( variance(u) )</pre>
    </blockquote>
    <p>
    The <a href=\"modelica://Modelica.Blocks.Math.Variance\">Variance</a> block is used to
    calculate variance(u).
    </p>
    <p>The parameter t_eps is used to guard against division by zero (the computation of the standard deviation
    starts at &lt;<em>simulation start time</em>&gt; + t_eps and before that time instant y = 0).
    </p>
    
    <p>
    This block is demonstrated in the examples
    <a href=\"modelica://Modelica.Blocks.Examples.NoiseExamples.UniformNoiseProperties\">UniformNoiseProperties</a> and
    <a href=\"modelica://Modelica.Blocks.Examples.NoiseExamples.NormalNoiseProperties\">NormalNoiseProperties</a>.
    </p>
    </html>"),
        Icon(graphics={
            Line(points={{-76,68},{-76,-80}}, color={192,192,192}),
            Line(points={{-86,0},{72,0}}, color={192,192,192}),
            Line(
               points={{-76,-13},{-62,-13},{-62,3},{-54,3},{-54,-45},{-46,-45},{-46,
                  -23},{-38,-23},{-38,61},{-30,61},{-30,29},{-30,29},{-30,-31},{-20,
                  -31},{-20,-13},{-10,-13},{-10,-41},{0,-41},{0,41},{6,41},{6,55},
                  {12,55},{12,-1},{22,-1},{22,11},{28,11},{28,-19},{38,-19},{38,53},
                  {48,53},{48,19},{56,19},{56,-47},{66,-47}},
                color={215,215,215}),
            Polygon(
              points={{94,0},{72,8},{72,-8},{94,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-76,46},{70,46}},
              color={215,215,215}),
            Line(
              points={{-16,0},{-16,30}}),
            Polygon(
              points={{-76,90},{-84,68},{-68,68},{-76,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-16,46},{-24,24},{-8,24},{-16,46}},
              fillPattern=FillPattern.Solid)}));
    end StandardDeviationWithReset;
  
    record Parameters
      Real K_co = 3.66e-5;
      Real K_ex = 0.717;
      Real K_soc = 0.916;
    end Parameters;
  
    parameter Real C_bat "Nominal capacity in Ah";
    parameter Parameters params;
    Modelica.Blocks.Logical.LessThreshold isCharging(threshold = 0);
    //Real cycleStartTime(start = 0);
    //Real cycleLen;
    //Real SoC_avg(start = 0);
    //Real SoC_dev(start = 0);
    //Real n_m(start = 0) "Effective number of throughput cycles";
    Real L(start = 0) "Life ageing parameter";
    Real stepCapacityFade(start = 0);
    Real capacityFade(start = 0);
    Modelica.Blocks.Interfaces.RealInput SoC annotation(
      Placement(visible = true, transformation(origin = {0, 118}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I annotation(
      Placement(visible = true, transformation(origin = {120, 0}, extent = {{-20, 20}, {20, -20}}, rotation = 180), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CapacityFadingCalculator.IntegratorWithReset Ah_throughput(y_start = 0) annotation(
      Placement(visible = true, transformation(origin = {10, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Abs I_abs annotation(
      Placement(visible = true, transformation(origin = {42, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Q_us(start = C_bat) annotation(
      Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180), iconTransformation(origin = {0, -110}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  MeanWithReset SoC_avg annotation(
      Placement(visible = true, transformation(origin = {-30, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  StandardDeviationWithReset SoC_dev annotation(
      Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product SoC_dev_normed annotation(
      Placement(visible = true, transformation(origin = {70, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant unit_of_soc_for_cycle(k = 3.4641016151)  annotation(
      Placement(visible = true, transformation(origin = {30, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division n_m annotation(
      Placement(visible = true, transformation(origin = {-30, -14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant cycle_charge(k = 2 * C_bat)  annotation(
      Placement(visible = true, transformation(origin = {10, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  
  equation
    connect(isCharging.u, I) annotation(
      Line);
    connect(isCharging.y, SoC_avg.reset);
    connect(isCharging.y, SoC_dev.reset);
    connect(isCharging.y, Ah_throughput.reset) annotation(
      Line);
    L = 1 - Q_us / C_bat;
    stepCapacityFade = params.K_co * n_m.y * exp((SoC_dev.y - 1) / params.K_ex) * exp(params.K_soc * ((SoC_avg.y - 0.5) / 0.25)) * (1 - L);
    Q_us = C_bat - capacityFade - stepCapacityFade;
    when isCharging.y == false then
      //cycleLen = time - pre(cycleStartTime);
      //SoC_avg = 1 / cycleLen * pre(socIntegrator.y);
      //SoC_dev = 2 * sqrt(3 / cycleLen) * sqrt(pre(socSquareIntegrator.y) - 2 * SoC_avg * pre(socIntegrator.y) + cycleLen * SoC_avg * SoC_avg);
      //n_m = pre(Ah_throughput.y) / (2 * C_bat);
      //Q_us = pre(Q_us) - stepCapacityFade;
      capacityFade = pre(capacityFade) + stepCapacityFade;
      //cycleStartTime = time;
    elsewhen isCharging.y == true then
      //cycleLen = pre(cycleLen);
      //SoC_avg = pre(SoC_avg);
      //SoC_dev = pre(SoC_dev);
      //n_m = pre(n_m);
      //stepCapacityFade = pre(stepCapacityFade);
      capacityFade = pre(capacityFade);
      //Q_us = pre(Q_us);
      //cycleStartTime = pre(cycleStartTime);
    end when;
    connect(I, I_abs.u) annotation(
      Line(points = {{120, 0}, {54, 0}}, color = {0, 0, 127}));
  connect(I_abs.y, Ah_throughput.u) annotation(
      Line(points = {{31, 0}, {22, 0}}, color = {0, 0, 127}));
  connect(SoC, SoC_avg.u) annotation(
      Line(points = {{0, 118}, {0, 70}, {-18, 70}}, color = {0, 0, 127}));
  connect(SoC, SoC_dev.u) annotation(
      Line(points = {{0, 118}, {0, 70}, {18, 70}}, color = {0, 0, 127}));
  connect(SoC_dev.y, SoC_dev_normed.u1) annotation(
      Line(points = {{41, 70}, {51.5, 70}, {51.5, 64}, {58, 64}}, color = {0, 0, 127}));
  connect(unit_of_soc_for_cycle.y, SoC_dev_normed.u2) annotation(
      Line(points = {{41, 44}, {52.5, 44}, {52.5, 52}, {58, 52}}, color = {0, 0, 127}));
  connect(Ah_throughput.y, n_m.u1) annotation(
      Line(points = {{-1, 0}, {-10.5, 0}, {-10.5, -8}, {-18, -8}}, color = {0, 0, 127}));
  connect(cycle_charge.y, n_m.u2) annotation(
      Line(points = {{0, -30}, {-10, -30}, {-10, -20}, {-18, -20}}, color = {0, 0, 127}));
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
    R = (params.R_0 + params.k1 * SoC + params.k2 * SoC ^ 2 + params.k3 * SoC ^ 3 + params.k4 * SoC ^ 4)*3600;
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
  parameter Real I_dis = C_bat;
  parameter String SocToOcvTableFileName = "/Users/roman.milishchuk/bachelor/soc_to_u_bat_tookup.txt";
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
  Modelica.Electrical.Analog.Basic.Ground GND annotation(
    Placement(visible = true, transformation(origin = {-86, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  CoulombSocCounter coulombSocCounter annotation(
    Placement(visible = true, transformation(origin = {30, -22}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  TestCycleTheveninBasedBattery.voltageSourceSocDependant voltageSource(SocToULookupTableFileName = SocToOcvTableFileName) annotation(
    Placement(visible = true, transformation(origin = {-86, -22}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  CapacityFadingCalculator capacityFadingCalc(C_bat = C_bat, params = capacityFadingParams) annotation(
    Placement(visible = true, transformation(origin = {4, -58}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.TableCurrent testCycleLoad(table = [0.0, 0.00033; 0.0, 0.000308; 0.0, 0.000297; 34.94, 0.000334; 69.99, 0.000334; 105.04, 0.000334; 140.08, 0.000333; 153.23, 0.000318; 153.23, 0.000305; 153.23, 0.000293; 153.23, 0.000281; 153.23, 0.000268; 153.23, 0.000256; 153.23, 0.000244; 153.23, 0.000231; 153.23, 0.000219; 153.23, 0.000207; 153.23, 0.000195; 153.23, 0.000182; 153.23, 0.00017; 153.23, 0.000158; 153.23, 0.000145; 153.23, 0.000133; 153.23, 0.000121; 153.23, 0.000108; 153.23, 9.6e-05; 153.23, 8.4e-05; 153.23, 7.2e-05; 153.23, 5.9e-05; 153.23, 4.7e-05; 153.23, 3.5e-05; 153.23, 2.2e-05; 163.74, 3e-06; 194.12, 4e-06; 197.04, 2.2e-05; 197.04, 3.5e-05; 197.04, 4.7e-05; 197.04, 5.9e-05; 197.04, 7.2e-05; 197.04, 8.4e-05; 197.04, 9.6e-05; 197.04, 0.000108; 197.04, 0.000121; 197.04, 0.000133; 197.04, 0.000145; 197.04, 0.000158; 197.04, 0.00017; 197.04, 0.000182; 197.04, 0.000195; 197.04, 0.000207; 197.04, 0.000219; 197.04, 0.000231; 197.04, 0.000244; 197.04, 0.000256; 197.04, 0.000268; 197.04, 0.000281; 197.04, 0.000293; 197.04, 0.000305; 197.04, 0.000318; 210.18, 0.000333; 245.23, 0.000334; 280.28, 0.000334; 315.33, 0.000334; 350.38, 0.000332; 359.14, 0.000318; 359.14, 0.000305; 359.14, 0.000293; 359.14, 0.000281; 359.14, 0.000268; 359.14, 0.000256; 359.14, 0.000244; 359.14, 0.000231; 359.14, 0.000219; 359.14, 0.000207; 359.14, 0.000195; 359.14, 0.000182; 359.14, 0.00017; 359.14, 0.000158; 359.14, 0.000145; 359.14, 0.000133; 359.14, 0.000121; 359.14, 0.000108; 359.14, 9.6e-05; 359.14, 8.4e-05; 359.14, 7.2e-05; 359.14, 5.9e-05; 359.14, 4.7e-05; 359.14, 3.5e-05; 359.14, 2.2e-05; 364.98, 4e-06; 402.95, 1e-06; 430.7, 4e-06; 433.62, 2.2e-05; 433.62, 3.5e-05; 433.62, 4.7e-05; 433.62, 5.9e-05; 433.62, 7.2e-05; 433.62, 8.4e-05; 433.62, 9.6e-05; 433.62, 0.000108; 433.62, 0.000121; 433.62, 0.000133; 433.62, 0.000145; 433.62, 0.000158; 433.62, 0.00017; 433.62, 0.000182; 433.62, 0.000195; 433.62, 0.000207; 433.62, 0.000219; 433.62, 0.000231; 433.62, 0.000244; 433.62, 0.000256; 433.62, 0.000268; 433.62, 0.000281; 433.62, 0.000293; 433.62, 0.000305; 433.62, 0.000318; 446.76, 0.000333; 481.81, 0.000334; 516.86, 0.000334; 551.91, 0.000334; 582.57, 0.000333; 591.34, 0.000318; 591.34, 0.000305; 591.34, 0.000293; 591.34, 0.000281; 591.34, 0.000268; 591.34, 0.000256; 591.34, 0.000244; 591.34, 0.000231; 591.34, 0.000219; 591.34, 0.000207; 591.34, 0.000195; 591.34, 0.000182; 591.34, 0.00017; 591.34, 0.000158; 591.34, 0.000145; 591.34, 0.000133; 591.34, 0.000121; 591.34, 0.000108; 591.34, 9.6e-05; 591.34, 8.4e-05; 591.34, 7.2e-05; 591.34, 5.9e-05; 591.34, 4.7e-05; 591.34, 3.5e-05; 591.34, 2.2e-05; 601.85, 3e-06; 639.53, 1e-06; 661.43, 6e-06; 661.43, 2.2e-05; 661.43, 3.5e-05; 661.43, 4.7e-05; 661.43, 5.9e-05; 661.43, 7.2e-05; 661.43, 8.4e-05; 661.43, 9.6e-05; 661.43, 0.000108; 661.43, 0.000121; 661.43, 0.000133; 661.43, 0.000145; 661.43, 0.000158; 661.43, 0.00017; 661.43, 0.000182; 661.43, 0.000195; 661.43, 0.000207; 661.43, 0.000219; 661.43, 0.000231; 661.43, 0.000244; 661.43, 0.000256; 661.43, 0.000268; 661.43, 0.000281; 661.43, 0.000293; 661.43, 0.000305; 661.43, 0.000318; 674.58, 0.000333; 709.62, 0.000334; 744.67, 0.000334; 779.72, 0.000334; 814.77, 0.000334; 849.82, 0.000334; 884.87, 0.000334; 919.92, 0.000334; 954.96, 0.000334; 981.25, 0.000331; 985.63, 0.000318; 985.63, 0.000305; 985.63, 0.000293; 985.63, 0.000281; 985.63, 0.000268; 985.63, 0.000256; 985.63, 0.000244; 985.63, 0.000231; 985.63, 0.000219; 985.63, 0.000207; 985.63, 0.000195; 985.63, 0.000182; 985.63, 0.00017; 985.63, 0.000158; 985.63, 0.000145; 985.63, 0.000133; 985.63, 0.000121; 985.63, 0.000108; 985.63, 9.6e-05; 985.63, 8.4e-05; 985.63, 7.2e-05; 985.63, 5.9e-05; 985.63, 4.7e-05; 985.63, 3.5e-05; 985.63, 2.2e-05; 996.15, 3e-06; 1031.63, 3e-06; 1038.21, 2.2e-05; 1038.21, 3.5e-05; 1038.21, 4.7e-05; 1038.21, 5.9e-05; 1038.21, 7.2e-05; 1038.21, 8.4e-05; 1038.21, 9.6e-05; 1038.21, 0.000108; 1038.21, 0.000121; 1038.21, 0.000133; 1038.21, 0.000145; 1038.21, 0.000158; 1038.21, 0.00017; 1038.21, 0.000182; 1038.21, 0.000195; 1038.21, 0.000207; 1038.21, 0.000219; 1038.21, 0.000231; 1038.21, 0.000244; 1038.21, 0.000256; 1038.21, 0.000268; 1038.21, 0.000281; 1038.21, 0.000293; 1038.21, 0.000305; 1038.21, 0.000318; 1051.35, 0.000333; 1086.4, 0.000334; 1121.45, 0.000334; 1156.49, 0.000334; 1191.54, 0.000334; 1226.59, 0.000334; 1261.64, 0.000334; 1296.69, 0.000334; 1331.74, 0.000334; 1366.79, 0.000333; 1384.31, 0.000304; 1384.31, 0.000291; 1384.31, 0.000279; 1384.31, 0.000267; 1384.31, 0.000255; 1384.31, 0.000242; 1384.31, 0.00023; 1384.31, 0.000218; 1384.31, 0.000205; 1384.31, 0.000193; 1384.31, 0.000181; 1384.31, 0.000168; 1384.31, 0.000156; 1384.31, 0.000144; 1384.31, 0.000132; 1384.31, 0.000119; 1384.31, 0.000107; 1384.31, 9.5e-05; 1384.31, 8.2e-05; 1384.31, 7e-05; 1384.31, 5.8e-05; 1384.31, 4.5e-05; 1384.31, 3.3e-05; 1384.31, 2.1e-05; 1390.44, 4e-06; 1401.83, 0.0; 1429.58, 4e-06; 1432.5, 2.2e-05; 1432.5, 3.5e-05; 1432.5, 4.7e-05; 1432.5, 5.9e-05; 1432.5, 7.2e-05; 1432.5, 8.4e-05; 1432.5, 9.6e-05; 1432.5, 0.000108; 1432.5, 0.000121; 1432.5, 0.000133; 1432.5, 0.000145; 1432.5, 0.000158; 1432.5, 0.00017; 1432.5, 0.000182; 1432.5, 0.000195; 1432.5, 0.000207; 1432.5, 0.000219; 1432.5, 0.000231; 1432.5, 0.000244; 1432.5, 0.000256; 1432.5, 0.000268; 1432.5, 0.000281; 1432.5, 0.000293; 1432.5, 0.000305; 1432.5, 0.000318; 1445.65, 0.000333; 1480.69, 0.000334; 1515.74, 0.000334; 1550.79, 0.000334; 1585.84, 0.000334; 1620.89, 0.000334; 1655.94, 0.000334; 1690.99, 0.000334; 1726.03, 0.000334; 1752.32, 0.00033; 1752.32, 0.000318; 1752.32, 0.000305; 1752.32, 0.000293; 1752.32, 0.000281; 1752.32, 0.000268; 1752.32, 0.000256; 1752.32, 0.000244; 1752.32, 0.000231; 1752.32, 0.000219; 1752.32, 0.000207; 1752.32, 0.000195; 1752.32, 0.000182; 1752.32, 0.00017; 1752.32, 0.000158; 1752.32, 0.000145; 1752.32, 0.000133; 1752.32, 0.000121; 1752.32, 0.000108; 1752.32, 9.6e-05; 1752.32, 8.4e-05; 1752.32, 7.2e-05; 1752.32, 5.9e-05; 1752.32, 4.7e-05; 1752.32, 3.5e-05; 1752.32, 2.2e-05; 1758.16, 4e-06; 1796.13, 1e-06; 1823.88, 4e-06; 1826.8, 2.2e-05; 1826.8, 3.5e-05; 1826.8, 4.7e-05; 1826.8, 5.9e-05; 1826.8, 7.2e-05; 1826.8, 8.4e-05; 1826.8, 9.6e-05; 1826.8, 0.000108; 1826.8, 0.000121; 1826.8, 0.000133; 1826.8, 0.000145; 1826.8, 0.000158; 1826.8, 0.00017; 1826.8, 0.000182; 1826.8, 0.000195; 1826.8, 0.000207; 1826.8, 0.000219; 1826.8, 0.000231; 1826.8, 0.000244; 1826.8, 0.000256; 1826.8, 0.000268; 1826.8, 0.000281; 1826.8, 0.000293; 1826.8, 0.000305; 1826.8, 0.000318; 1839.94, 0.000333; 1874.99, 0.000334; 1910.04, 0.000334; 1945.09, 0.000334; 1980.14, 0.000334; 2015.19, 0.000334; 2050.23, 0.000334; 2085.28, 0.000334; 2120.33, 0.000334; 2146.62, 0.000328; 2146.62, 0.000316; 2146.62, 0.000304; 2146.62, 0.000291; 2146.62, 0.000279; 2146.62, 0.000267; 2146.62, 0.000255; 2146.62, 0.000242; 2146.62, 0.00023; 2146.62, 0.000218; 2146.62, 0.000205; 2146.62, 0.000193; 2146.62, 0.000181; 2146.62, 0.000168; 2146.62, 0.000156; 2146.62, 0.000144; 2146.62, 0.000132; 2146.62, 0.000119; 2146.62, 0.000107; 2146.62, 9.5e-05; 2146.62, 8.2e-05; 2146.62, 7e-05; 2146.62, 5.8e-05; 2146.62, 4.5e-05; 2146.62, 3.3e-05; 2146.62, 2.1e-05; 2152.75, 4e-06; 2164.14, 0.0; 2201.82, 2e-06; 2216.71, 1.5e-05; 2216.71, 2.7e-05; 2216.71, 3.9e-05; 2216.71, 5.2e-05; 2216.71, 6.4e-05; 2216.71, 7.6e-05; 2216.71, 8.9e-05; 2216.71, 0.000101; 2216.71, 0.000113; 2216.71, 0.000125; 2216.71, 0.000138; 2216.71, 0.00015; 2216.71, 0.000162; 2216.71, 0.000175; 2216.71, 0.000187; 2216.71, 0.000199; 2216.71, 0.000211; 2216.71, 0.000224; 2216.71, 0.000236; 2216.71, 0.000248; 2216.71, 0.000261; 2216.71, 0.000273; 2216.71, 0.000285; 2216.71, 0.000298; 2216.71, 0.00031; 2224.6, 0.000329; 2260.53, 0.000334; 2295.57, 0.000334; 2330.62, 0.000334; 2365.67, 0.000334; 2400.72, 0.000334; 2435.77, 0.000334; 2470.82, 0.000334; 2505.87, 0.000334; 2536.53, 0.000333; 2545.3, 0.000318; 2545.3, 0.000305; 2545.3, 0.000293; 2545.3, 0.000281; 2545.3, 0.000268; 2545.3, 0.000256; 2545.3, 0.000244; 2545.3, 0.000231; 2545.3, 0.000219; 2545.3, 0.000207; 2545.3, 0.000195; 2545.3, 0.000182; 2545.3, 0.00017; 2545.3, 0.000158; 2545.3, 0.000145; 2545.3, 0.000133; 2545.3, 0.000121; 2545.3, 0.000108; 2545.3, 9.6e-05; 2545.3, 8.4e-05; 2545.3, 7.2e-05; 2545.3, 5.9e-05; 2545.3, 4.7e-05; 2545.3, 3.5e-05; 2545.3, 2.2e-05; 2555.81, 3e-06; 2596.12, 3e-06; 2606.63, 2.2e-05; 2606.63, 3.5e-05; 2606.63, 4.7e-05; 2606.63, 5.9e-05; 2606.63, 7.2e-05; 2606.63, 8.4e-05; 2606.63, 9.6e-05; 2606.63, 0.000108; 2606.63, 0.000121; 2606.63, 0.000133; 2606.63, 0.000145; 2606.63, 0.000158; 2606.63, 0.00017; 2606.63, 0.000182; 2606.63, 0.000195; 2606.63, 0.000207; 2606.63, 0.000219; 2606.63, 0.000231; 2606.63, 0.000244; 2606.63, 0.000256; 2606.63, 0.000268; 2606.63, 0.000281; 2606.63, 0.000293; 2606.63, 0.000305; 2606.63, 0.000318; 2619.77, 0.000333; 2654.82, 0.000334; 2689.87, 0.000334; 2724.92, 0.000334; 2759.97, 0.000334; 2795.02, 0.000334; 2830.07, 0.000334; 2865.11, 0.000334; 2900.16, 0.000334; 2930.83, 0.000333; 2939.59, 0.000318; 2939.59, 0.000305; 2939.59, 0.000293; 2939.59, 0.000281; 2939.59, 0.000268; 2939.59, 0.000256; 2939.59, 0.000244; 2939.59, 0.000231; 2939.59, 0.000219; 2939.59, 0.000207; 2939.59, 0.000195; 2939.59, 0.000182; 2939.59, 0.00017; 2939.59, 0.000158; 2939.59, 0.000145; 2939.59, 0.000133; 2939.59, 0.000121; 2939.59, 0.000108; 2939.59, 9.6e-05; 2939.59, 8.4e-05; 2939.59, 7.2e-05; 2939.59, 5.9e-05; 2939.59, 4.7e-05; 2939.59, 3.5e-05; 2939.59, 2.2e-05; 2950.11, 3e-06; 2987.78, 1e-06; 3022.83, 1e-06; 3050.58, 4e-06; 3053.5, 2.2e-05; 3053.5, 3.5e-05; 3053.5, 4.7e-05; 3053.5, 5.9e-05; 3053.5, 7.2e-05; 3053.5, 8.4e-05; 3053.5, 9.6e-05; 3053.5, 0.000108; 3053.5, 0.000121; 3053.5, 0.000133; 3053.5, 0.000145; 3053.5, 0.000158; 3053.5, 0.00017; 3053.5, 0.000182; 3053.5, 0.000195; 3053.5, 0.000207; 3053.5, 0.000219; 3053.5, 0.000231; 3053.5, 0.000244; 3053.5, 0.000256; 3053.5, 0.000268; 3053.5, 0.000281; 3053.5, 0.000293; 3053.5, 0.000305; 3053.5, 0.000318; 3066.64, 0.000333; 3101.69, 0.000334; 3136.74, 0.000334; 3171.79, 0.000334; 3206.84, 0.000333; 3219.98, 0.000318; 3219.98, 0.000305; 3219.98, 0.000293; 3219.98, 0.000281; 3219.98, 0.000268; 3219.98, 0.000256; 3219.98, 0.000244; 3219.98, 0.000231; 3219.98, 0.000219; 3219.98, 0.000207; 3219.98, 0.000195; 3219.98, 0.000182; 3219.98, 0.00017; 3219.98, 0.000158; 3219.98, 0.000145; 3219.98, 0.000133; 3219.98, 0.000121; 3219.98, 0.000108; 3219.98, 9.6e-05; 3219.98, 8.4e-05; 3219.98, 7.2e-05; 3219.98, 5.9e-05; 3219.98, 4.7e-05; 3219.98, 3.5e-05; 3219.98, 2.2e-05; 3230.5, 3e-06; 3270.8, 2e-06; 3285.7, 1.5e-05; 3285.7, 2.7e-05; 3285.7, 3.9e-05; 3285.7, 5.2e-05; 3285.7, 6.4e-05; 3285.7, 7.6e-05; 3285.7, 8.9e-05; 3285.7, 0.000101; 3285.7, 0.000113; 3285.7, 0.000125; 3285.7, 0.000138; 3285.7, 0.00015; 3285.7, 0.000162; 3285.7, 0.000175; 3285.7, 0.000187; 3285.7, 0.000199; 3285.7, 0.000211; 3285.7, 0.000224; 3285.7, 0.000236; 3285.7, 0.000248; 3285.7, 0.000261; 3285.7, 0.000273; 3285.7, 0.000285; 3285.7, 0.000298; 3285.7, 0.00031; 3293.58, 0.000329; 3329.51, 0.000334; 3364.56, 0.000334; 3399.61, 0.000334; 3434.65, 0.000333; 3447.8, 0.000318; 3447.8, 0.000305; 3447.8, 0.000293; 3447.8, 0.000281; 3447.8, 0.000268; 3447.8, 0.000256; 3447.8, 0.000244; 3447.8, 0.000231; 3447.8, 0.000219; 3447.8, 0.000207; 3447.8, 0.000195; 3447.8, 0.000182; 3447.8, 0.00017; 3447.8, 0.000158; 3447.8, 0.000145; 3447.8, 0.000133; 3447.8, 0.000121; 3447.8, 0.000108; 3447.8, 9.6e-05; 3447.8, 8.4e-05; 3447.8, 7.2e-05; 3447.8, 5.9e-05; 3447.8, 4.7e-05; 3447.8, 3.5e-05; 3447.8, 2.2e-05; 3458.31, 3e-06; 3491.61, 4e-06; 3495.99, 1.8e-05; 3495.99, 3e-05; 3495.99, 4.2e-05; 3495.99, 5.5e-05; 3495.99, 6.7e-05; 3495.99, 7.9e-05; 3495.99, 9.2e-05; 3495.99, 0.000104; 3495.99, 0.000116; 3495.99, 0.000128; 3495.99, 0.000141; 3495.99, 0.000153; 3495.99, 0.000165; 3495.99, 0.000178; 3495.99, 0.00019; 3495.99, 0.000202; 3495.99, 0.000215; 3495.99, 0.000227; 3495.99, 0.000239; 3495.99, 0.000251; 3495.99, 0.000264; 3495.99, 0.000276; 3495.99, 0.000288; 3495.99, 0.000301; 3495.99, 0.000313; 3503.88, 0.00033; 3539.8, 0.000334; 3574.85, 0.000334; 3609.9, 0.000334; 3644.95, 0.000333; 3662.47, 0.000304; 3662.47, 0.000291; 3662.47, 0.000279; 3662.47, 0.000267; 3662.47, 0.000255; 3662.47, 0.000242; 3662.47, 0.00023; 3662.47, 0.000218; 3662.47, 0.000205; 3662.47, 0.000193; 3662.47, 0.000181; 3662.47, 0.000168; 3662.47, 0.000156; 3662.47, 0.000144; 3662.47, 0.000132; 3662.47, 0.000119; 3662.47, 0.000107; 3662.47, 9.5e-05; 3662.47, 8.2e-05; 3662.47, 7e-05; 3662.47, 5.8e-05; 3662.47, 4.5e-05; 3662.47, 3.3e-05; 3662.47, 2.1e-05; 3668.6, 4e-06; 3679.99, 0.0; 3717.67, 3e-06; 3728.19, 2.2e-05; 3728.19, 3.5e-05; 3728.19, 4.7e-05; 3728.19, 5.9e-05; 3728.19, 7.2e-05; 3728.19, 8.4e-05; 3728.19, 9.6e-05; 3728.19, 0.000108; 3728.19, 0.000121; 3728.19, 0.000133; 3728.19, 0.000145; 3728.19, 0.000158; 3728.19, 0.00017; 3728.19, 0.000182; 3728.19, 0.000195; 3728.19, 0.000207; 3728.19, 0.000219; 3728.19, 0.000231; 3728.19, 0.000244; 3728.19, 0.000256; 3728.19, 0.000268; 3728.19, 0.000281; 3728.19, 0.000293; 3728.19, 0.000305; 3728.19, 0.000318; 3741.33, 0.000333; 3776.38, 0.000334; 3798.28, 0.00033; 3798.28, 0.000318; 3798.28, 0.000305; 3798.28, 0.000293; 3798.28, 0.000281; 3798.28, 0.000268; 3798.28, 0.000256; 3798.28, 0.000244; 3798.28, 0.000231; 3798.28, 0.000219; 3798.28, 0.000207; 3798.28, 0.000195; 3798.28, 0.000182; 3798.28, 0.00017; 3802.66, 0.000154; 3820.19, 0.000142; 3824.57, 0.000127; 3824.57, 0.000115; 3824.57, 0.000102; 3824.57, 9e-05; 3828.95, 7.5e-05; 3850.86, 6.4e-05; 3881.52, 4.9e-05; 3916.57, 4e-05; 3947.24, 3.4e-05; 3977.91, 2.7e-05; 4012.96, 2.5e-05; 4048.0, 1.8e-05; 4083.05, 1.8e-05; 4118.1, 1.8e-05; 4153.15, 1.6e-05; 4188.2, 1e-05; 4223.25, 1e-05; 4258.3, 1e-05; 4293.34, 1e-05; 4328.39, 1e-05; 4363.44, 1e-05; 4398.49, 1e-05; 4433.54, 1e-05; 4468.59, 1e-05; 4503.64, 1e-05; 4538.69, 1e-05; 4573.73, 1e-05; 4608.78, 1e-05; 4643.83, 1e-05; 4678.88, 1e-05; 4713.93, 1e-05; 4748.98, 1e-05; 4784.03, 1e-05; 4819.07, 1e-05; 4854.12, 1e-05; 4889.17, 1e-05; 4924.22, 1e-05; 4959.27, 1e-05; 4994.32, 1e-05; 5029.37, 1e-05; 5064.41, 1e-05; 5099.46, 5e-06; 5134.51, 1e-06; 5169.56, 1e-06; 5204.61, 1e-06; 5239.66, 0.0; 5274.71, 1e-06; 5309.75, 1e-06; 5344.8, 1e-06; 5379.85, 1e-06; 5414.9, 1e-06; 5449.95, 1e-06; 5485.0, 1e-06; 5520.05, 1e-06; 5555.09, 1e-06; 5590.14, 1e-06; 5625.19, 1e-06; 5660.24, 1e-06; 5695.29, 1e-06; 5730.34, 1e-06; 5765.39, 1e-06; 5800.44, 1e-06; 5835.48, 1e-06; 5870.53, 1e-06; 5905.58, 1e-06; 5940.63, 1e-06; 5975.68, 1e-06; 6010.73, 1e-06; 6045.78, 1e-06; 6080.82, 1e-06; 6115.87, 1e-06; 6150.92, 1e-06; 6185.97, 1e-06; 6221.02, 1e-06; 6256.07, 1e-06; 6291.12, 1e-06; 6326.16, 1e-06; 6361.21, 1e-06; 6396.26, 1e-06; 6431.31, 1e-06; 6466.36, 1e-06; 6501.41, 1e-06; 6536.46, 1e-06; 6571.5, -1e-06; 6584.65, -1.8e-05; 6584.65, -3e-05; 6584.65, -4.2e-05; 6584.65, -5.4e-05; 6584.65, -6.7e-05; 6584.65, -7.9e-05; 6584.65, -9.1e-05; 6584.65, -0.000104; 6584.65, -0.000116; 6584.65, -0.000128; 6584.65, -0.00014; 6584.65, -0.000153; 6584.65, -0.000165; 6584.65, -0.000177; 6584.65, -0.00019; 6584.65, -0.000202; 6584.65, -0.000214; 6584.65, -0.000227; 6584.65, -0.000239; 6584.65, -0.000251; 6584.65, -0.000263; 6584.65, -0.000276; 6584.65, -0.000288; 6584.65, -0.0003; 6584.65, -0.000313; 6584.65, -0.00033; 6606.55, -0.000336; 6641.6, -0.000336; 6676.65, -0.000336; 6711.7, -0.000336; 6739.45, -0.000331; 6742.37, -0.000313; 6742.37, -0.0003; 6742.37, -0.000288; 6742.37, -0.000276; 6742.37, -0.000263; 6742.37, -0.000251; 6742.37, -0.000239; 6742.37, -0.000227; 6742.37, -0.000214; 6742.37, -0.000202; 6742.37, -0.00019; 6742.37, -0.000177; 6742.37, -0.000165; 6742.37, -0.000153; 6742.37, -0.00014; 6742.37, -0.000128; 6742.37, -0.000116; 6742.37, -0.000104; 6742.37, -9.1e-05; 6742.37, -7.9e-05; 6742.37, -6.7e-05; 6742.37, -5.4e-05; 6742.37, -4.2e-05; 6742.37, -3e-05; 6742.37, -1.8e-05; 6755.51, -1e-06; 6790.56, 1e-06; 6812.46, -5e-06; 6812.46, -1.8e-05; 6812.46, -3e-05; 6812.46, -4.2e-05; 6812.46, -5.4e-05; 6812.46, -6.7e-05; 6812.46, -7.9e-05; 6812.46, -9.1e-05; 6812.46, -0.000104; 6812.46, -0.000116; 6812.46, -0.000128; 6812.46, -0.00014; 6812.46, -0.000153; 6812.46, -0.000165; 6812.46, -0.000177; 6812.46, -0.00019; 6812.46, -0.000202; 6812.46, -0.000214; 6812.46, -0.000227; 6812.46, -0.000239; 6812.46, -0.000251; 6812.46, -0.000263; 6812.46, -0.000276; 6812.46, -0.000288; 6812.46, -0.0003; 6812.46, -0.000313; 6815.38, -0.000331; 6843.13, -0.000336; 6878.18, -0.000336; 6913.23, -0.000336; 6948.28, -0.000336; 6973.1, -0.000332; 6974.56, -0.00032; 6974.56, -0.000308; 6974.56, -0.000296; 6974.56, -0.000283; 6974.56, -0.000271; 6974.56, -0.000259; 6974.56, -0.000247; 6974.56, -0.000234; 6974.56, -0.000222; 6974.56, -0.00021; 6974.56, -0.000197; 6974.56, -0.000185; 6974.56, -0.000173; 6974.56, -0.00016; 6974.56, -0.000148; 6974.56, -0.000136; 6974.56, -0.000124; 6974.56, -0.000111; 6974.56, -9.9e-05; 6974.56, -8.7e-05; 6974.56, -7.4e-05; 6974.56, -6.2e-05; 6974.56, -5e-05; 6974.56, -3.8e-05; 6974.56, -2.5e-05; 6982.45, -6e-06; 7018.37, -1e-06; 7031.52, -1.8e-05; 7031.52, -3e-05; 7031.52, -4.2e-05; 7031.52, -5.4e-05; 7031.52, -6.7e-05; 7031.52, -7.9e-05; 7031.52, -9.1e-05; 7031.52, -0.000104; 7031.52, -0.000116; 7031.52, -0.000128; 7031.52, -0.00014; 7031.52, -0.000153; 7031.52, -0.000165; 7031.52, -0.000177; 7031.52, -0.00019; 7031.52, -0.000202; 7031.52, -0.000214; 7031.52, -0.000227; 7031.52, -0.000239; 7031.52, -0.000251; 7031.52, -0.000263; 7031.52, -0.000276; 7031.52, -0.000288; 7031.52, -0.0003; 7031.52, -0.000313; 7031.52, -0.00033; 7053.42, -0.000336; 7088.47, -0.000336; 7123.52, -0.000336; 7158.57, -0.000336; 7186.32, -0.000331; 7189.24, -0.000313; 7189.24, -0.0003; 7189.24, -0.000288; 7189.24, -0.000276; 7189.24, -0.000263; 7189.24, -0.000251; 7189.24, -0.000239; 7189.24, -0.000227; 7189.24, -0.000214; 7189.24, -0.000202; 7189.24, -0.00019; 7189.24, -0.000177; 7189.24, -0.000165; 7189.24, -0.000153; 7189.24, -0.00014; 7189.24, -0.000128; 7189.24, -0.000116; 7189.24, -0.000104; 7189.24, -9.1e-05; 7189.24, -7.9e-05; 7189.24, -6.7e-05; 7189.24, -5.4e-05; 7189.24, -4.2e-05; 7189.24, -3e-05; 7189.24, -1.8e-05; 7202.38, -1e-06; 7237.43, 1e-06; 7259.33, -5e-06; 7259.33, -1.8e-05; 7259.33, -3e-05; 7259.33, -4.2e-05; 7259.33, -5.4e-05; 7259.33, -6.7e-05; 7259.33, -7.9e-05; 7259.33, -9.1e-05; 7259.33, -0.000104; 7259.33, -0.000116; 7259.33, -0.000128; 7259.33, -0.00014; 7259.33, -0.000153; 7259.33, -0.000165; 7259.33, -0.000177; 7259.33, -0.00019; 7259.33, -0.000202; 7259.33, -0.000214; 7259.33, -0.000227; 7259.33, -0.000239; 7259.33, -0.000251; 7259.33, -0.000263; 7259.33, -0.000276; 7259.33, -0.000288; 7259.33, -0.0003; 7259.33, -0.000313; 7262.25, -0.000331; 7290.0, -0.000336; 7325.05, -0.000336; 7360.1, -0.000336; 7395.15, -0.000336; 7430.2, -0.000336; 7465.24, -0.000336; 7500.29, -0.000336; 7535.34, -0.000336; 7570.39, -0.000335; 7583.53, -0.000325; 7583.53, -0.000313; 7583.53, -0.0003; 7583.53, -0.000288; 7583.53, -0.000276; 7583.53, -0.000263; 7583.53, -0.000251; 7583.53, -0.000239; 7583.53, -0.000227; 7583.53, -0.000214; 7583.53, -0.000202; 7583.53, -0.00019; 7583.53, -0.000177; 7583.53, -0.000165; 7583.53, -0.000153; 7583.53, -0.00014; 7583.53, -0.000128; 7583.53, -0.000116; 7583.53, -0.000104; 7583.53, -9.1e-05; 7583.53, -7.9e-05; 7583.53, -6.7e-05; 7583.53, -5.4e-05; 7583.53, -4.2e-05; 7583.53, -3e-05; 7583.53, -1.8e-05; 7596.68, -1e-06; 7631.73, 1e-06; 7653.63, -5e-06; 7653.63, -1.8e-05; 7653.63, -3e-05; 7653.63, -4.2e-05; 7653.63, -5.4e-05; 7653.63, -6.7e-05; 7653.63, -7.9e-05; 7653.63, -9.1e-05; 7653.63, -0.000104; 7653.63, -0.000116; 7653.63, -0.000128; 7653.63, -0.00014; 7653.63, -0.000153; 7653.63, -0.000165; 7653.63, -0.000177; 7653.63, -0.00019; 7653.63, -0.000202; 7653.63, -0.000214; 7653.63, -0.000227; 7653.63, -0.000239; 7653.63, -0.000251; 7653.63, -0.000263; 7653.63, -0.000276; 7653.63, -0.000288; 7653.63, -0.0003; 7653.63, -0.000313; 7656.55, -0.000331; 7684.3, -0.000336; 7719.35, -0.000336; 7754.4, -0.000336; 7789.44, -0.000336; 7824.49, -0.000336; 7859.54, -0.000336; 7894.59, -0.000336; 7929.64, -0.000336; 7964.69, -0.000335; 7977.83, -0.000325; 7977.83, -0.000313; 7977.83, -0.0003; 7977.83, -0.000288; 7977.83, -0.000276; 7977.83, -0.000263; 7977.83, -0.000251; 7977.83, -0.000239; 7977.83, -0.000227; 7977.83, -0.000214; 7977.83, -0.000202; 7977.83, -0.00019; 7977.83, -0.000177; 7977.83, -0.000165; 7977.83, -0.000153; 7977.83, -0.00014; 7977.83, -0.000128; 7977.83, -0.000116; 7977.83, -0.000104; 7977.83, -9.1e-05; 7977.83, -7.9e-05; 7977.83, -6.7e-05; 7977.83, -5.4e-05; 7977.83, -4.2e-05; 7977.83, -3e-05; 7977.83, -1.8e-05; 7990.97, -1e-06; 8026.02, 1e-06; 8047.93, -5e-06; 8047.93, -1.8e-05; 8047.93, -3e-05; 8047.93, -4.2e-05; 8047.93, -5.4e-05; 8047.93, -6.7e-05; 8047.93, -7.9e-05; 8047.93, -9.1e-05; 8047.93, -0.000104; 8047.93, -0.000116; 8047.93, -0.000128; 8047.93, -0.00014; 8047.93, -0.000153; 8047.93, -0.000165; 8047.93, -0.000177; 8047.93, -0.00019; 8047.93, -0.000202; 8047.93, -0.000214; 8047.93, -0.000227; 8047.93, -0.000239; 8047.93, -0.000251; 8047.93, -0.000263; 8047.93, -0.000276; 8047.93, -0.000288; 8047.93, -0.0003; 8047.93, -0.000313; 8050.85, -0.000331; 8078.59, -0.000336; 8113.64, -0.000336; 8148.69, -0.000336; 8183.74, -0.000336; 8218.79, -0.000336; 8253.84, -0.000336; 8288.89, -0.000336; 8323.94, -0.000336; 8359.86, -0.000334; 8367.75, -0.00032; 8367.75, -0.000308; 8367.75, -0.000296; 8367.75, -0.000283; 8367.75, -0.000271; 8367.75, -0.000259; 8367.75, -0.000247; 8367.75, -0.000234; 8367.75, -0.000222; 8367.75, -0.00021; 8367.75, -0.000197; 8367.75, -0.000185; 8367.75, -0.000173; 8367.75, -0.00016; 8367.75, -0.000148; 8367.75, -0.000136; 8367.75, -0.000124; 8367.75, -0.000111; 8367.75, -9.9e-05; 8367.75, -8.7e-05; 8367.75, -7.4e-05; 8367.75, -6.2e-05; 8367.75, -5e-05; 8367.75, -3.8e-05; 8367.75, -2.5e-05; 8375.63, -6e-06; 8411.56, -1e-06; 8424.7, -1.8e-05; 8424.7, -3e-05; 8424.7, -4.2e-05; 8424.7, -5.4e-05; 8424.7, -6.7e-05; 8424.7, -7.9e-05; 8424.7, -9.1e-05; 8424.7, -0.000104; 8424.7, -0.000116; 8424.7, -0.000128; 8424.7, -0.00014; 8424.7, -0.000153; 8424.7, -0.000165; 8424.7, -0.000177; 8424.7, -0.00019; 8424.7, -0.000202; 8424.7, -0.000214; 8424.7, -0.000227; 8424.7, -0.000239; 8424.7, -0.000251; 8424.7, -0.000274; 8429.08, -0.000268; 8433.46, -0.000285; 8433.46, -0.000297; 8436.38, -0.000315; 8455.37, -0.000335; 8490.42, -0.000336; 8525.46, -0.000336; 8560.51, -0.000336; 8595.56, -0.000336; 8630.61, -0.000336; 8665.66, -0.000336; 8700.71, -0.000336; 8735.76, -0.000336; 8766.42, -0.000333; 8766.42, -0.000325; 8770.81, -0.000317; 8770.81, -0.000305; 8770.81, -0.000293; 8770.81, -0.00028; 8770.81, -0.000268; 8770.81, -0.000256; 8770.81, -0.000243; 8770.81, -0.000231; 8770.81, -0.000219; 8770.81, -0.000207; 8770.81, -0.000194; 8770.81, -0.000182; 8770.81, -0.00017; 8770.81, -0.000157; 8770.81, -0.000145; 8770.81, -0.000133; 8770.81, -0.000121; 8770.81, -0.000108; 8770.81, -9.6e-05; 8770.81, -8.4e-05; 8770.81, -7.1e-05; 8770.81, -5.9e-05; 8770.81, -4.7e-05; 8770.81, -3.4e-05; 8770.81, -2.2e-05; 8778.69, -4e-06; 8810.23, -3e-06; 8810.23, -1.8e-05; 8810.23, -3e-05; 8810.23, -4.2e-05; 8810.23, -5.4e-05; 8810.23, -6.7e-05; 8810.23, -7.9e-05; 8810.23, -9.1e-05; 8810.23, -0.000104; 8810.23, -0.000116; 8810.23, -0.000128; 8810.23, -0.00014; 8810.23, -0.000153; 8810.23, -0.000165; 8810.23, -0.000177; 8810.23, -0.00019; 8810.23, -0.000202; 8810.23, -0.000214; 8810.23, -0.000227; 8810.23, -0.000239; 8810.23, -0.000251; 8810.23, -0.000263; 8810.23, -0.000276; 8810.23, -0.000288; 8810.23, -0.0003; 8810.23, -0.000313; 8813.16, -0.00033; 8840.9, -0.000336; 8875.95, -0.000336; 8911.0, -0.000336; 8946.05, -0.000336; 8981.1, -0.000336; 9016.15, -0.000336; 9051.19, -0.000336; 9086.24, -0.000336; 9121.29, -0.000335; 9134.43, -0.000325; 9134.43, -0.000313; 9134.43, -0.0003; 9134.43, -0.000288; 9134.43, -0.000276; 9134.43, -0.000263; 9134.43, -0.000251; 9134.43, -0.000239; 9134.43, -0.000227; 9134.43, -0.000214; 9134.43, -0.000202; 9134.43, -0.00019; 9134.43, -0.000177; 9134.43, -0.000165; 9134.43, -0.000153; 9134.43, -0.00014; 9134.43, -0.000128; 9134.43, -0.000116; 9134.43, -0.000104; 9134.43, -9.1e-05; 9134.43, -7.9e-05; 9134.43, -6.7e-05; 9134.43, -5.4e-05; 9134.43, -4.2e-05; 9134.43, -3e-05; 9134.43, -1.8e-05; 9147.58, -1e-06; 9182.63, 1e-06; 9208.91, -7e-06; 9208.91, -1.9e-05; 9208.91, -3.1e-05; 9208.91, -4.4e-05; 9208.91, -5.6e-05; 9208.91, -6.8e-05; 9208.91, -8.1e-05; 9208.91, -9.3e-05; 9208.91, -0.000105; 9208.91, -0.000117; 9208.91, -0.00013; 9208.91, -0.000142; 9208.91, -0.000154; 9208.91, -0.000167; 9208.91, -0.000179; 9208.91, -0.000191; 9208.91, -0.000204; 9208.91, -0.000216; 9208.91, -0.000228; 9208.91, -0.00024; 9208.91, -0.000253; 9208.91, -0.000265; 9208.91, -0.000277; 9208.91, -0.00029; 9208.91, -0.000302; 9208.91, -0.000314; 9207.45, -0.000329; 9226.44, -0.000336; 9261.49, -0.000336; 9296.53, -0.000336; 9331.58, -0.000336; 9366.63, -0.000336; 9401.68, -0.000336; 9436.73, -0.000336; 9471.78, -0.000336; 9506.83, -0.000336; 9534.57, -0.000331; 9537.49, -0.000313; 9537.49, -0.0003; 9537.49, -0.000288; 9537.49, -0.000276; 9537.49, -0.000263; 9537.49, -0.000251; 9537.49, -0.000239; 9537.49, -0.000227; 9537.49, -0.000214; 9537.49, -0.000202; 9537.49, -0.00019; 9537.49, -0.000177; 9537.49, -0.000165; 9537.49, -0.000153; 9537.49, -0.00014; 9537.49, -0.000128; 9537.49, -0.000116; 9537.49, -0.000104; 9537.49, -9.1e-05; 9537.49, -7.9e-05; 9537.49, -6.7e-05; 9537.49, -5.4e-05; 9537.49, -4.2e-05; 9537.49, -3e-05; 9537.49, -1.8e-05; 9550.64, -1e-06; 9585.69, 1e-06; 9620.73, 1e-06; 9647.02, -2e-06; 9651.4, -1.8e-05; 9651.4, -3e-05; 9651.4, -4.2e-05; 9651.4, -5.4e-05; 9651.4, -6.7e-05; 9651.4, -7.9e-05; 9651.4, -9.1e-05; 9651.4, -0.000104; 9651.4, -0.000116; 9651.4, -0.000128; 9651.4, -0.00014; 9651.4, -0.000153; 9651.4, -0.000165; 9651.4, -0.000177; 9651.4, -0.00019; 9651.4, -0.000202; 9651.4, -0.000214; 9651.4, -0.000227; 9651.4, -0.000239; 9651.4, -0.000251; 9651.4, -0.000263; 9651.4, -0.000276; 9651.4, -0.000288; 9651.4, -0.0003; 9651.4, -0.000313; 9654.32, -0.000331; 9682.07, -0.000336; 9717.12, -0.000336; 9752.17, -0.000336; 9787.21, -0.000336; 9814.96, -0.000331; 9817.88, -0.000313; 9817.88, -0.0003; 9817.88, -0.000288; 9817.88, -0.000276; 9817.88, -0.000263; 9817.88, -0.000251; 9817.88, -0.000239; 9817.88, -0.000227; 9817.88, -0.000214; 9817.88, -0.000202; 9817.88, -0.00019; 9817.88, -0.000177; 9817.88, -0.000165; 9817.88, -0.000153; 9817.88, -0.00014; 9817.88, -0.000128; 9817.88, -0.000116; 9817.88, -0.000104; 9817.88, -9.1e-05; 9817.88, -7.9e-05; 9817.88, -6.7e-05; 9817.88, -5.4e-05; 9817.88, -4.2e-05; 9817.88, -3e-05; 9817.88, -1.8e-05; 9831.03, -1e-06; 9861.69, -1e-06; 9883.6, -1.3e-05; 9883.6, -2.5e-05; 9883.6, -3.8e-05; 9883.6, -5e-05; 9883.6, -6.2e-05; 9883.6, -7.4e-05; 9883.6, -8.7e-05; 9883.6, -9.9e-05; 9883.6, -0.000111; 9883.6, -0.000124; 9883.6, -0.000136; 9883.6, -0.000148; 9883.6, -0.00016; 9883.6, -0.000173; 9883.6, -0.000185; 9883.6, -0.000197; 9883.6, -0.00021; 9883.6, -0.000222; 9883.6, -0.000234; 9883.6, -0.000247; 9883.6, -0.000259; 9883.6, -0.000271; 9883.6, -0.000283; 9883.6, -0.000296; 9883.6, -0.000308; 9882.14, -0.000323; 9901.12, -0.000335; 9936.17, -0.000336; 9971.22, -0.000336; 10006.27, -0.000336; 10036.94, -0.000335; 10045.7, -0.000325; 10045.7, -0.000313; 10045.7, -0.0003; 10045.7, -0.000288; 10045.7, -0.000276; 10045.7, -0.000263; 10045.7, -0.000251; 10045.7, -0.000239; 10045.7, -0.000227; 10045.7, -0.000214; 10045.7, -0.000202; 10045.7, -0.00019; 10045.7, -0.000177; 10045.7, -0.000165; 10045.7, -0.000153; 10045.7, -0.00014; 10045.7, -0.000128; 10045.7, -0.000116; 10045.7, -0.000104; 10045.7, -9.1e-05; 10045.7, -7.9e-05; 10045.7, -6.7e-05; 10045.7, -5.4e-05; 10045.7, -4.2e-05; 10045.7, -3e-05; 10045.7, -1.8e-05; 10058.84, -1e-06; 10092.43, -1e-06; 10115.8, -1.4e-05; 10115.8, -2.7e-05; 10115.8, -3.9e-05; 10115.8, -5.1e-05; 10115.8, -6.4e-05; 10115.8, -7.6e-05; 10115.8, -8.8e-05; 10115.8, -0.000101; 10115.8, -0.000113; 10115.8, -0.000125; 10115.8, -0.000137; 10115.8, -0.00015; 10115.8, -0.000162; 10115.8, -0.000174; 10115.8, -0.000187; 10115.8, -0.000199; 10115.8, -0.000211; 10115.8, -0.000223; 10115.8, -0.000236; 10115.8, -0.000248; 10115.8, -0.00026; 10115.8, -0.000273; 10115.8, -0.000285; 10115.8, -0.000297; 10115.8, -0.00031; 10115.8, -0.000326; 10137.7, -0.000335; 10168.37, -0.000332; 10190.27, -0.000323; 10203.42, -0.000314; 10203.42, -0.000303; 10203.42, -0.000291; 10203.42, -0.000279; 10207.8, -0.000262; 10225.32, -0.000251; 10234.08, -0.000241; 10235.54, -0.000227; 10247.23, -0.000211; 10253.07, -0.0002; 10269.13, -0.000187; 10282.28, -0.000177; 10286.66, -0.000161; 10308.56, -0.000151; 10326.09, -0.00014; 10326.09, -0.000128; 10326.09, -0.000116; 10326.09, -0.000104; 10326.09, -9.1e-05; 10326.09, -7.9e-05; 10326.09, -6.7e-05; 10326.09, -5.4e-05; 10326.09, -4.2e-05; 10326.09, -3e-05; 10326.09, -1.8e-05; 10339.23, -1e-06; 10365.52, -2e-06; 10369.9, -1.8e-05; 10369.9, -3e-05; 10369.9, -4.2e-05; 10369.9, -5.4e-05; 10374.28, -6.7e-05; 10374.28, -7.9e-05; 10374.28, -9.1e-05; 10378.66, -0.000104; 10378.66, -0.000116; 10378.66, -0.000128; 10378.66, -0.00014; 10383.04, -0.000156; 10383.04, -0.000168; 10383.04, -0.00018; 10383.04, -0.000193; 10390.34, -0.000205; 10393.26, -0.000218; 10391.8, -0.000234; 10391.8, -0.000247; 10391.8, -0.000259; 10391.8, -0.000271; 10391.8, -0.000283; 10391.8, -0.000296; 10391.8, -0.000308; 10391.8, -0.00032; 10391.8, -0.00033; 10413.71, -0.0002; 10413.71, -0.00019; 10413.71, -0.000177; 10413.71, -0.000165; 10413.71, -0.000153; 10413.71, -0.00014; 10422.47, -0.000123; 10442.92, -0.00011; 10466.28, -9.4e-05; 10488.19, -7.8e-05; 10514.47, -6.9e-05; 10549.52, -6.1e-05; 10584.57, -5.9e-05; 10619.62, -5.1e-05; 10654.67, -4.7e-05; 10689.72, -4.1e-05; 10724.77, -3.8e-05; 10759.81, -3.7e-05; 10794.86, -2.9e-05; 10829.91, -2.8e-05; 10864.96, -2.8e-05; 10900.01, -2.8e-05; 10935.06, -2.1e-05; 10970.11, -2.1e-05; 11005.15, -2.1e-05; 11040.2, -2.1e-05; 11075.25, -2.1e-05; 11110.3, -2.1e-05; 11145.35, -2e-05; 11180.4, -1.2e-05; 11215.45, -1e-05; 11250.49, -1e-05; 11285.54, -1e-05; 11320.59, -1e-05; 11355.64, -1e-05; 11390.69, -1e-05; 11425.74, -1e-05; 11460.79, -1e-05; 11495.83, -1e-05; 11530.88, -1e-05; 11565.93, -1e-05; 11600.98, -1e-05; 11636.03, -1e-05; 11671.08, -1e-05; 11706.13, -1e-05; 11741.17, -1e-05; 11776.22, -1e-05; 11811.27, -1e-05; 11846.32, -1e-05; 11881.37, -1e-05; 11916.42, -1e-05; 11951.47, -1e-05; 11986.52, -5e-06; 12021.56, -3e-06; 12056.61, 1e-06; 12091.66, 1e-06; 12126.71, 1e-06; 12161.76, 1e-06; 12196.81, 1e-06; 12231.86, 1e-06; 12266.9, 1e-06; 12301.95, 1e-06; 12323.86, 1e-06; 3663.93, 0.00032; 3290.08, 2e-06; 2221.1, 2e-06; 1385.77, 0.00032]) annotation(
    Placement(visible = true, transformation(origin = {34, -90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
    Placement(visible = true, transformation(origin = {-74, 72}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
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
  connect(capacityFadingCalc.Q_us, coulombSocCounter.Q_us) annotation(
    Line(points = {{-6, -58}, {-20, -58}, {-20, -74}, {70, -74}, {70, -18}, {40, -18}, {40, -18}}, color = {0, 0, 127}));
  connect(testCycleLoad.n, GND.p) annotation(
    Line(points = {{24, -90}, {-86, -90}}, color = {0, 0, 255}));
  connect(I_bat.n, testCycleLoad.p) annotation(
    Line(points = {{70, 40}, {80, 40}, {80, -90}, {44, -90}, {44, -90}, {44, -90}}, color = {0, 0, 255}));
  connect(voltageSensor.p, I_bat.n) annotation(
    Line(points = {{-64, 72}, {70, 72}, {70, 40}}, color = {0, 0, 255}));
  connect(voltageSensor.n, GND.p) annotation(
    Line(points = {{-84, 72}, {-98, 72}, {-98, -90}, {-86, -90}}, color = {0, 0, 255}));
protected
  annotation(
    Diagram(coordinateSystem(initialScale = 0.1)),
    uses(Modelica(version = "3.2.3")),
    Documentation,
    experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-06, Interval = 100));
end TestCycleTheveninBasedBattery;
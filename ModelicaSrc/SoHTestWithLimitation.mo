package BatteryMPC
  model BatteryWithFullCycle
    model TheveninBasedBattery "Basic Battery Model based on Thevenin electrical model"
      extends Modelica.Electrical.Analog.Interfaces.TwoPin;
  
      model VariableSource "Generate signal of type Real"
        Real height(start=0) "Height of step" annotation(
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
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{-50, 50}, {50, -50}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-150, 60}, {150, 100}}, textString = "%name", lineColor = {0, 0, 255}), Line(points = {{-90, 0}, {90, 0}}, color = {0, 0, 255}), Line(points = {{-80, 20}, {-60, 20}}, color = {0, 0, 255}), Line(points = {{-70, 30}, {-70, 10}}, color = {0, 0, 255}), Line(points = {{60, 20}, {80, 20}}, color = {0, 0, 255})}),
          Documentation(revisions = "<html>
      <ul>
      <li><em> 1998   </em>
           by Christoph Clauss<br> initially implemented<br>
           </li>
      </ul>
      </html>", info = "<html>
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
        R = 3600 * (params.R_0 + params.k1 * exp(-params.k2 * SoC) + params.k3 * SoC);
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
        C = max(params.C_0 + params.k1 * SoC + params.k2 * SoC ^ 2 + params.k3 * SoC ^ 3 + params.k4 * SoC ^ 4 + params.k5 * SoC ^ 5 + params.k6 * SoC ^ 6, 0) / 3600;
      end ChargeDependentCapacitor;
  
      block CoulombSocCounter
        Modelica.Blocks.Interfaces.RealInput I_bat(start=0) "Connector of Real input signals" annotation(
          Placement(visible = true, transformation(origin = {120, 26}, extent = {{-20, 20}, {20, -20}}, rotation = 180), iconTransformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
        Modelica.Blocks.Sources.Constant SOC_init(k = 0.95) annotation(
          Placement(visible = true, transformation(origin = {17, 49}, extent = {{11, -11}, {-11, 11}}, rotation = 0)));
        Modelica.Blocks.Math.Sum Sum(k = {1, -1}, nin = 2) annotation(
          Placement(visible = true, transformation(origin = {-26, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput SoC annotation(
          Placement(visible = true, transformation(origin = {-110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Q_us annotation(
          Placement(visible = true, transformation(origin = {120, -26}, extent = {{20, -20}, {-20, 20}}, rotation = 0), iconTransformation(origin = {-40, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
        Modelica.Blocks.Math.Division division annotation(
          Placement(visible = true, transformation(origin = {14, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator annotation(
          Placement(visible = true, transformation(origin = {60, 24}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      equation
        connect(SOC_init.y, Sum.u[1]) annotation(
          Line(points = {{5, 49}, {-4, 49}, {-4, 0}, {-14, 0}}, color = {0, 0, 127}));
        connect(Q_us, division.u2) annotation(
          Line(points = {{120, -26}, {26, -26}}, color = {0, 0, 127}));
        connect(division.y, Sum.u[2]) annotation(
          Line(points = {{3, -20}, {-4.5, -20}, {-4.5, 0}, {-14, 0}}, color = {0, 0, 127}));
        connect(integrator.y, division.u1) annotation(
          Line(points = {{50, 24}, {34, 24}, {34, -14}, {26, -14}}, color = {0, 0, 127}));
        connect(integrator.u, I_bat) annotation(
          Line(points = {{72, 24}, {88, 24}, {88, 26}, {120, 26}}, color = {0, 0, 127}));
  connect(Sum.y, SoC) annotation(
          Line(points = {{-36, 0}, {-110, 0}}, color = {0, 0, 127}));
        annotation(
          uses(Modelica(version = "3.2.3")),
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "")}),
        Documentation(info="<html>
      <p>
      Block that has only the basic icon for an input/output
      block (no declarations, no equations). Most blocks
      of package Modelica.Blocks inherit directly or indirectly
      from this block.
      </p>
      </html>"));
      end CoulombSocCounter;
  
      block CapacityFadingCalculator
        model IntegratorWithReset
          extends Modelica.Blocks.Continuous.Integrator;
          Modelica.Blocks.Interfaces.BooleanInput reset;
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
          Real mu(start=0) "Mean value (state variable)";
          Real var(start=0) "Variance (state variable)";
          Real t_0(start=0) "Start time";
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
          Real mu(start=0) "Internal integrator variable";
          Real t_0(start=0) "Start time";
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
      
        parameter Real C_bat = 1 "Nominal capacity in Ah";
        parameter Parameters params;
        Modelica.Blocks.Logical.GreaterThreshold isNotCharging(threshold = 0);
        Real L(start = 0) "Life ageing parameter";
        Real stepCapacityFade(start = 0);
        Real capacityFade(start = 0);
        Modelica.Blocks.Interfaces.RealInput SoC annotation(
          Placement(visible = true, transformation(origin = {0, 118}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput I annotation(
          Placement(visible = true, transformation(origin = {120, 0}, extent = {{-20, 20}, {20, -20}}, rotation = 180), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
        Modelica.Blocks.Continuous.Integrator Ah_throughput(use_reset = true, y_start = 0) annotation(
          Placement(visible = true, transformation(origin = {10, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Abs I_abs annotation(
          Placement(visible = true, transformation(origin = {42, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Q_us(start = C_bat, fixed=true) annotation(
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
        connect(isNotCharging.u, I) annotation(
          Line);
        connect(isNotCharging.y, SoC_avg.reset);
        connect(isNotCharging.y, SoC_dev.reset);
        connect(isNotCharging.y, Ah_throughput.reset) annotation(
          Line);
        Q_us = C_bat - capacityFade - stepCapacityFade;
        L = 1 - Q_us / C_bat;
        stepCapacityFade = params.K_co * n_m.y * exp((SoC_dev_normed.y - 1) / params.K_ex) * exp(params.K_soc * ((SoC_avg.y - 0.5) / 0.25)) * (1 - L);
        when isNotCharging.y then
          capacityFade = pre(capacityFade) + pre(stepCapacityFade);
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
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,150},{150,110}},
              lineColor={0,0,255})}),
        Documentation(info="<html>
      <p>
      Block that has only the basic icon for an input/output
      block (no declarations, no equations). Most blocks
      of package Modelica.Blocks inherit directly or indirectly
      from this block.
      </p>
      </html>"));
      
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
        R = (params.R_0 + params.k1 * SoC + params.k2 * SoC ^ 2 + params.k3 * SoC ^ 3 + params.k4 * SoC ^ 4) * 3600;
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
      parameter Real C_bat = 1;
      parameter String SocToOcvTableFileName = "/home/developer/modelica/soc_to_u_bat_tookup.txt";
      parameter CapacityFadingCalculator.Parameters capacityFadingParams(K_co = 3.66e-5, K_ex = 0.717, K_soc = 0.916);
      Modelica.Blocks.Interfaces.RealOutput SoH(start = 1);
      Real SoH_last(start = 1);
      Real SoH_diff(start=0) annotation(
        Placement(visible = true, transformation(origin = {40, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {36, 94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
        Placement(visible = true, transformation(origin = {34, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
      BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.voltageSourceSocDependant U_oc(SocToULookupTableFileName = SocToOcvTableFileName) annotation(
        Placement(visible = true, transformation(origin = {-76, 0}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
      BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery.CapacityFadingCalculator capacityFadingCalc(C_bat = C_bat, params = capacityFadingParams) annotation(
        Placement(visible = true, transformation(origin = {-10, -82}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput SoC annotation(
        Placement(visible = true, transformation(origin = {-40, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-56, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      SoH = 1 - (capacityFadingCalc.capacityFade + capacityFadingCalc.stepCapacityFade) / (0.2 * C_bat);
      SoH_last = delay(SoH, 1);
      SoH_diff = SoH - SoH_last;
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
      connect(U_oc.p, R_s.n) annotation(
        Line(points = {{-66, 0}, {-66, 40}, {-58, 40}}, color = {0, 0, 255}));
  connect(coulombSocCounter.SoC, U_oc.SoC) annotation(
        Line(points = {{23, -58}, {-75.75, -58}, {-75.75, -12}, {-76, -12}}, color = {0, 0, 127}));
  connect(coulombSocCounter.SoC, capacityFadingCalc.SoC) annotation(
        Line(points = {{23, -58}, {-10, -58}, {-10, -72}}, color = {0, 0, 127}));
  connect(capacityFadingCalc.I, I_bat.i) annotation(
        Line(points = {{0, -82}, {60, -82}, {60, 30}}, color = {0, 0, 127}));
  connect(capacityFadingCalc.Q_us, coulombSocCounter.Q_us) annotation(
        Line(points = {{-21, -82}, {-36, -82}, {-36, -36}, {53, -36}, {53, -54}, {44, -54}}, color = {0, 0, 127}));
      connect(p, U_oc.n) annotation(
        Line(points = {{-100, 0}, {-86, 0}, {-86, 0}}, color = {0, 0, 255}));
      connect(I_bat.n, n) annotation(
        Line(points = {{70, 40}, {80, 40}, {80, 0}, {100, 0}}, color = {0, 0, 255}));
      connect(C_tl.p, R_tl.p) annotation(
        Line(points = {{40, 12}, {40, 40}}, color = {0, 0, 255}));
  connect(coulombSocCounter.SoC, SoC) annotation(
        Line(points = {{24, -58}, {6, -58}, {6, 60}, {-40, 60}, {-40, 110}}, color = {0, 0, 127}));
    protected
      annotation(
        Diagram(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {0.599966, 23.2454}, extent = {{140.429, -43.2454}, {-140.429, 43.2454}}), Text(origin = {-102, 55}, extent = {{-28, 5}, {28, -5}}, textString = "Thevenin-based model"), Text(origin = {-18, -96}, extent = {{-18, -2}, {18, 2}}, textString = "Q_us calculation"), Text(origin = {20, -74}, extent = {{-18, -2}, {18, 2}}, textString = "SoC calculation")}),
        uses(Modelica(version = "3.2.3")),
        Documentation,
        experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-06, Interval = 100));
    end TheveninBasedBattery;
  
    parameter Real C_bat = 1.1;
    parameter Real I_dis = C_bat / 3600;
    BatteryMPC.BatteryWithFullCycle.TheveninBasedBattery theveninBasedBattery(C_bat = 1, SocToOcvTableFileName = "/Users/roman.milishchuk/bachelor/ModelicaSrc/soc_to_u_bat_tookup.txt") annotation(
      Placement(visible = true, transformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-50, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalCurrent pulseLoad annotation(
      Placement(visible = true, transformation(origin = {0, -16}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SoC annotation(
      Placement(visible = true, transformation(origin = {-40, 110}, extent = {{10, -10}, {-10, 10}}, rotation = -90), iconTransformation(origin = {-38, 94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SoH annotation(
      Placement(visible = true, transformation(origin = {40, 110}, extent = {{-10, 10}, {10, -10}}, rotation = 90), iconTransformation(origin = {62, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor batteryVoltageSensor annotation(
      Placement(visible = true, transformation(origin = {0, 58}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput batteryOutput annotation(
      Placement(visible = true, transformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {2, 104}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch1 annotation(
      Placement(visible = true, transformation(origin = {40, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold = 0.99)  annotation(
      Placement(visible = true, transformation(origin = {-66, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold = 0)  annotation(
      Placement(visible = true, transformation(origin = {-66, -94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.MathBoolean.And and1(nu = 2)  annotation(
      Placement(visible = true, transformation(origin = {-18, -132}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.MathBoolean.Not not1 annotation(
      Placement(visible = true, transformation(origin = {16, -130}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch11 annotation(
      Placement(visible = true, transformation(origin = {110, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold = 0.01)  annotation(
      Placement(visible = true, transformation(origin = {70, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold = 0)  annotation(
      Placement(visible = true, transformation(origin = {72, -126}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.MathBoolean.And and11(nu = 2)  annotation(
      Placement(visible = true, transformation(origin = {122, -114}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.MathBoolean.Not not11 annotation(
      Placement(visible = true, transformation(origin = {150, -116}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constZero(k = 0)  annotation(
      Placement(visible = true, transformation(origin = {-4, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse pulse(amplitude = 0.00085, offset = -0.00031, period = 4600, startTime = 0, width = 37)  annotation(
      Placement(visible = true, transformation(origin = {-256, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(ground.p, theveninBasedBattery.p) annotation(
      Line(points = {{-50, -16}, {-50, 20}, {-10, 20}}, color = {0, 0, 255}));
    connect(pulseLoad.n, ground.p) annotation(
      Line(points = {{-10, -16}, {-50, -16}}, color = {0, 0, 255}));
    connect(theveninBasedBattery.n, pulseLoad.p) annotation(
      Line(points = {{10, 20}, {20, 20}, {20, -16}, {10, -16}}, color = {0, 0, 255}));
    connect(theveninBasedBattery.SoC, SoC) annotation(
      Line(points = {{-6, 30}, {-6, 80}, {-40, 80}, {-40, 110}}, color = {0, 0, 127}));
    connect(theveninBasedBattery.SoH, SoH) annotation(
      Line(points = {{4, 30}, {4, 80}, {40, 80}, {40, 110}}, color = {0, 0, 127}));
    connect(batteryVoltageSensor.v, batteryOutput) annotation(
      Line(points = {{0, 69}, {0, 110}}, color = {0, 0, 127}));
    connect(batteryVoltageSensor.n, theveninBasedBattery.p) annotation(
      Line(points = {{-10, 58}, {-10, 20}}, color = {0, 0, 255}));
    connect(batteryVoltageSensor.p, theveninBasedBattery.n) annotation(
      Line(points = {{10, 58}, {10, 20}}, color = {0, 0, 255}));
    connect(theveninBasedBattery.SoC, greaterEqualThreshold.u) annotation(
      Line(points = {{-6, 30}, {-4, 30}, {-4, 44}, {-94, 44}, {-94, -66}, {-78, -66}}, color = {0, 0, 127}));
    connect(greaterEqualThreshold.y, and1.u[1]) annotation(
      Line(points = {{-54, -66}, {-30, -66}, {-30, -132}, {-24, -132}}, color = {255, 0, 255}));
    connect(lessThreshold.y, and1.u[2]) annotation(
      Line(points = {{-54, -94}, {-40, -94}, {-40, -132}, {-24, -132}}, color = {255, 0, 255}));
    connect(and1.y, not1.u) annotation(
      Line(points = {{-12, -132}, {10, -132}, {10, -130}}, color = {255, 0, 255}));
    connect(not1.y, switch1.u2) annotation(
      Line(points = {{20, -130}, {32, -130}, {32, -82}, {20, -82}, {20, -62}, {28, -62}}, color = {255, 0, 255}));
    connect(theveninBasedBattery.SoC, lessEqualThreshold.u) annotation(
      Line(points = {{-6, 30}, {-4, 30}, {-4, 42}, {-126, 42}, {-126, -112}, {44, -112}, {44, -96}, {58, -96}}, color = {0, 0, 127}));
    connect(lessEqualThreshold.y, and11.u[1]) annotation(
      Line(points = {{82, -96}, {96, -96}, {96, -114}, {116, -114}}, color = {255, 0, 255}));
    connect(greaterThreshold.y, and11.u[2]) annotation(
      Line(points = {{84, -126}, {98, -126}, {98, -114}, {116, -114}}, color = {255, 0, 255}));
    connect(and11.y, not11.u) annotation(
      Line(points = {{128, -114}, {138, -114}, {138, -116}, {144, -116}}, color = {255, 0, 255}));
    connect(not11.y, switch11.u2) annotation(
      Line(points = {{154, -116}, {168, -116}, {168, -74}, {82, -74}, {82, -58}, {98, -58}}, color = {255, 0, 255}));
    connect(switch11.y, pulseLoad.i) annotation(
      Line(points = {{122, -58}, {132, -58}, {132, -38}, {0, -38}, {0, -28}}, color = {0, 0, 127}));
    connect(constZero.y, switch1.u3) annotation(
      Line(points = {{8, -100}, {12, -100}, {12, -70}, {28, -70}}, color = {0, 0, 127}));
    connect(constZero.y, switch11.u3) annotation(
      Line(points = {{8, -100}, {38, -100}, {38, -78}, {88, -78}, {88, -66}, {98, -66}}, color = {0, 0, 127}));
    connect(switch1.y, switch11.u1) annotation(
      Line(points = {{52, -62}, {64, -62}, {64, -50}, {98, -50}}, color = {0, 0, 127}));
  connect(pulse.y, lessThreshold.u) annotation(
      Line(points = {{-244, -58}, {-176, -58}, {-176, -94}, {-78, -94}}, color = {0, 0, 127}));
  connect(pulse.y, switch1.u1) annotation(
      Line(points = {{-244, -58}, {-110, -58}, {-110, -46}, {20, -46}, {20, -54}, {28, -54}}, color = {0, 0, 127}));
  connect(pulse.y, greaterThreshold.u) annotation(
      Line(points = {{-244, -58}, {-208, -58}, {-208, -144}, {48, -144}, {48, -126}, {60, -126}}, color = {0, 0, 127}));
    annotation(
      uses(Modelica(version = "3.2.3")),
      experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-6, Interval = 100),
  Diagram);
  end BatteryWithFullCycle;
  annotation(
    uses(Modelica(version = "3.2.3")));
end BatteryMPC;
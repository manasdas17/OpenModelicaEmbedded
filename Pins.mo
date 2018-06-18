  package Pins "Components to access the board I/O"
    extends Internal.Icons.Block;

    model AnalogInput "Reads an analog signal from the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Number of the analog pin";
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(
        Placement(visible = true, transformation(origin = {-100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Real InitValue = 0 "Initial value until the board responds" annotation(
        Dialog(group = "Initialization"));
      parameter Real MinValue = 0 "Minimum value of ADC" annotation(
        Dialog(group = "Scaling"));
      parameter Real MaxValue = 1 "Maximum value of ADC" annotation(
        Dialog(group = "Scaling"));
      parameter Integer adcResolution = 10 "Resolution of the ADC in your microcontroller" annotation(
        Dialog(group = "Scaling"));
    equation
      y = OpenModelicaArduino.Internal.ExternalFunctions.readAnalogPin(Pin, MinValue, MaxValue, InitValue, pinConnector, adcResolution);
      annotation(
        Documentation(info = "<html><p>Reads an analog signal from the specified pin. This component uses the 'analogRead' function of Arduino.</p>
 <p><strong>Signal Range:</strong> By default, the signal goes from 0 to 1 where 0 represents no voltage and 1 the voltage reference of the ADC in the board. This signal can be scaled by setting the 'MinValue' and 'MaxValue' parameters.</p>
 <p>Not all pins support analog input. Check the documentation of your board to find the pin capabilities.</p>
 <p>&nbsp;</p></html>", revisions = ""),
        Icon(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10}, extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}),
        Diagram(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20}, extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}));
    end AnalogInput;

    model AnalogOutput "Writes an analog signal to the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-110, -0}, extent = {{-25, -25}, {25, 25}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Number of the PWM/DAC pin";
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Real MinValue = 0 "Value considered as minimum by the PWM/DAC" annotation(
        Dialog(group = "Scaling"));
      parameter Real MaxValue = 1 "Value considered as maximum by the PWM/DAC" annotation(
        Dialog(group = "Scaling"));
    protected
      Real scaled_u = (u - MinValue) / (MaxValue - MinValue);
    equation
      OpenModelicaArduino.Internal.ExternalFunctions.writeAnalogPin(Pin, pinConnector, scaled_u);
      annotation(
        Documentation(info = "<html><p>Writes an analog signal to the specified pin. This component uses the 'analogWrite' function of Arduino.</p>
 <p><strong>Signal Range:</strong> By default, the signal goes from 0 to 1, where 0 represents no voltage and 1 the maximum voltage that PWM/DAC of your board can provide. This signal can be scaled by setting the 'MinValue' and 'MaxValue' parameters.</p>
 <p>Analog outputs use the PWM capabilities of the pins, therefore, they do not provide a continuous signal. If you want to get a continuous signal, you need to add a lowpass filter. Check the Arduino Playground (<a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a>) for more information on PWM.</p>
 <p>Not all pins support analog output. Check the documentation of your board to find the pin capabilities.</p>
 <p>&nbsp;</p></html>", revisions = ""),
        Icon(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10}, extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Output", textStyle = {TextStyle.Bold})}),
        Diagram(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20}, extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Output", textStyle = {TextStyle.Bold})}));
    end AnalogOutput;

    model DigitalInput "Reads a digital signal from the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.BooleanOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 1.643}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {101.75, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Number of the digital pin";
      parameter Boolean InitValue = false "Initial value until the board responds" annotation(
        Dialog(group = "Initialization"));
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      y = OpenModelicaArduino.Internal.ExternalFunctions.readDigitalPin(Pin, InitValue, pinConnector);
      annotation(
        Documentation(info = "<html><p>Reads a digital signal from&nbsp;the specified pin. This component uses the 'digitalRead' function of Arduino.</p>
      <p><strong>Signal Range:</strong>&nbsp;This component accepts only Boolean signals (true/false).</p></html>", revisions = ""),
        Icon(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10}, extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}),
        Diagram(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20}, extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}));
    end DigitalInput;

    model DigitalOutput "Writes a digital signal to the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.BooleanInput u annotation(
        Placement(visible = true, transformation(origin = {-110, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Number of the digital pin";
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      OpenModelicaArduino.Internal.ExternalFunctions.writeDigitalPin(Pin, pinConnector, u);
      annotation(
        Documentation(info = "<html><p>Writes a digital signal to the specified pin. This component uses the 'digitalWrite' function of Arduino.</p>
     <p><strong>Signal Range:</strong>&nbsp;This component accepts only Boolean signals (true/false).</p></html>", revisions = ""),
        Icon(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10}, extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Output", textStyle = {TextStyle.Bold})}),
        Diagram(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20}, extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}));
    end DigitalOutput;

    model Servo "Controls a servo motor attached to the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Pin number of the servo";
      parameter OpenModelicaArduino.Internal.Types.ServoUnit InputUnit "if None (Default) the servo receives a signal between 0 and 1. If Degrees the control signal is from 0 to 180. If Radians the signal is from 0 to Pi.";
      parameter Integer MinPulse = 544 "The pulse width, in microseconds, corresponding to the minimum (0-degree) angle on the servo.
                  (Ignore this parameter if using Tiva C board)" annotation(
        Dialog(group = "Advanced"));
      parameter Integer MaxPulse = 2400 "The pulse width, in microseconds, corresponding to the maximum (180-degree) angle on the servo.
                  (Ignore this parameter if using Tiva C board)" annotation(
        Dialog(group = "Advanced"));
      Real outputSignal;
    equation
      outputSignal = if InputUnit == OpenModelicaArduino.Internal.Types.ServoUnit.None then u else if InputUnit == OpenModelicaArduino.Internal.Types.ServoUnit.Degrees then u / 180 else if InputUnit == OpenModelicaArduino.Internal.Types.ServoUnit.Radians then u / Modelica.Constants.pi else u;
      OpenModelicaArduino.Internal.ExternalFunctions.writeServoPin(Pin, pinConnector, u, MinPulse, MaxPulse);
      annotation(
        Documentation(info = "<html><p>Controls a servo motor attached to the specified pin. This component uses the 'Servo' library of Arduino.</p>
      <p><strong>Signal Range:</strong> By default, the range goes from 0 to 1, which corresponds to 0 to 180 degrees. If you want to input values in radians, you can change the parameter 'InputUnit' from 'Degrees' to 'Radians'.</p>
      <p>If your servo does not work correctly with the default settings, you can set the parameters 'MinPulse' and 'MaxPulse'. To get more information on how to configure a servo, you can check the documentation of the Servo library,&nbsp;<a href=\"http://arduino.cc/en/reference/servo\">http://arduino.cc/en/reference/servo</a>.</p></html>", revisions = ""),
        Icon(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {0, 255, 127}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(extent = {{-75, -25}, {75, 25}}, textString = "Servo", textStyle = {TextStyle.Bold})}),
        Diagram(coordinateSystem(initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {0, 255, 127}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(extent = {{-75, -25}, {75, 25}}, textString = "Servo", textStyle = {TextStyle.Bold})}));
    end Servo;
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {-30, 30}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50), Rectangle(visible = true, origin = {30, 30}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50), Rectangle(visible = true, origin = {-30, -30}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50), Rectangle(visible = true, origin = {30, -30}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50)}),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end Pins;
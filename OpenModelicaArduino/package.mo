package OpenModelicaArduino "Connecting OpenModelica with Arduino"
  package GettingStarted "Procedure to run first model using OpenModelicaArduino"
    extends Modelica.Icons.Information;
    annotation(preferredView = "info", Documentation(info = "<p> .</p>
      <h4>Preparing Your Board</h4>
      <p> .</p>
      <p>The first thing you need to do is upload the Firmware code to your board .</p>
      <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/FirmataLocation.png\" alt=\"\" /></p>
      <p><em>Figure 1. Location of the Firmware sketch.</em></p>
      <p>Once the the Firmware code is in the board, you&nbsp;need to write down the serial port that it is using. This is important because you&nbsp;need to give the port name to OpenModelicaArduino in order to communicate with the board. You can find the serial port in Tools-&gt;Serial Port or in the bottom-right corner of the Arduino software window (see Figure 2). In Windows the serial port name is something like &ldquo;COM5&rdquo;, while in OS X and Linux the name will be something like &ldquo;/dev/ttyACM0&rdquo;. Now you&nbsp;are ready to make your&nbsp;first model.</p>
      <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/SerialPortLocation.png\" alt=\"\" /></p>
      <p><em>Figure 2. Finding the serial port being used.</em>&nbsp;</p>
      <h4>Blinking LED</h4>
      <p>As a first exercise, you&nbsp;are going to reproduce with OpenModelicaArduino the blinking LED example. You can either open the prebuilt example (OpenModelicaArduino.Examples.BlinkLed) or build it by yourself. To build the model, locate the components:</p>
      <ul>
      <li>OpenModelicaArduino.Pins.DigitalOutput</li>
      <li>OpenModelicaArduino.Boards.Arduino</li>
      <li>Modelica.Blocks.Sources.BooleanPulse</li>
      </ul>
      <p>Connect the components as in Figure 3.&nbsp;One thing to notice is that the DigitalOutput model is connected to the Arduino component to specify that the pin belongs to that board. This connection is necessary because OpenModelicaArduino can use multiple boards with multiple pins at the same time.</p>
      <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/BlinkingLEDModel.png\" alt=\"\" /></p>
      <p><em>Figure 3. Diagram of the blinking LED.</em></p>
      <p>Next you&nbsp;need to specify the serial port that the board is using. This is done by selecting the Arduino component and showing its parameters. In the parameter view, you&nbsp;can find the Port parameter. Write the port name that you got in the section &ldquo;Preparing your board&rdquo;. Important: the name must be have quotation marks&nbsp;as shown in Figure 4.</p>
      <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/QuotedSerialPort.png\" alt=\"\" /></p>
      <p><em>Figure 4. Specifying the serial port name.&nbsp;</em></p>
      <p>Now you&nbsp;need to set the pin number that you&nbsp;are going to use in the DigitalOutput component. Usually, the Arduino boards have an LED attached to pin 13. Set that number in the Pin parameter as shown in Figure 5.</p>
      <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/PinNumber.png\" alt=\"\" /></p>
      <p><em>Figure 5. Specifying the pin to use.</em></p>
      <p>For the BooleanPulse component, set the 'period' parameter to 1. The model is ready to simulate. Press the simulate button and wait to see the results.</p>
      <p>The first time you run the model, it is probably simulated so fast that you do not have&nbsp;time to react. The reason is that OpenModelica simulates the model as fast as possible. In order to interact with your models using OpenModelicaArduino, it is necessary to synchronize the simulation time with real time. This is done in by using Synchronisation block in ModelicaDeviceDriver library. After simulating the model one time, you need to check in the checkbox&nbsp;&ldquo;Synchronize with real-time&rdquo; as shown in Figure 6.</p>
      //<p>&nbsp;<img src=\"Modelica://OpenModelicaArduino/Resources/Images/SynchronizeSetting.png\" alt=\"\" /></p>
      //<p><em>Figure 6. Synchronizing your simulation with real time.</em>&nbsp;</p>
      //<p>If you are building many models with the OpenModelicaArduino library, you should set the &ldquo;Synchronize with real-time&rdquo; option as a default simulation setting in SimulationCenter under the menu Tools-&gt;Options in the section SimulationCenter-&gt;Default Experiment (see Figure 7).&nbsp;</p>
      //<p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/DefaultSynchronize.png\" alt=\"\" />&nbsp;</p>
      //<p><em>Figure 7. Setting \"Synchronize with real-time\" as default.</em></p>
      //<p>Now run the simulation again and you should see the LED blinking until the simulation reaches the stop time. If you want to keep the simulation running continuously, you need to change the stop time to &ldquo;infinite&rdquo; as shown in Figure 8.</p>
      //<p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/StopTime.png\" alt=\"\" /></p>
      //<p><em>Figure 8. &nbsp;Setting the stop time to infinite.</em></p>
      //<p>If you run the simulation again, you should see the LED blinking continuously.</p>
      <p>One thing that you may have noticed is that OpenModelicaArduino prints status messages in the simulation log (see Figure 9). The first thing it prints is a list of the available ports. In that list, you should see your current port (A). After that, it prints the current port and the speed used (B). Once the port is opened, you&nbsp;receive a notification that the board is initialized (C). Usually the boards report the version of Firmata that you are running. Then you&nbsp;set the sampling interval that the board uses (D). In this example, you can see that you are setting pin 13 to be an output because you&nbsp;have used the DigitalOutput component (E). Finally you&nbsp;will see that the board will send you&nbsp;a list of all the pins available and thier capabilities (F). This list contains the number of the pin and the modes in which it can be used, for example: DigitalInput, DigitalOutput, AnalogInput, AnalogOutput, and Servo.</p>
      <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/ModelPlugLog-withmarkers.png\" alt=\"\" /></p>
      <p><em>Figure 9. Messages shown by OpenModelicaArduino.</em></p>
      <h4>Troubleshooting</h4>
      <p>This is a checklist that you can follow in order to solve most of the problems that can occur when using the ModelPlug library:</p>
      <ol>
      <li>Verify that you can upload any Arduino example to your board.</li>
      <li>Verify that you have uploaded a StandardFirmata example to your board.</li>
      <li>Verify that the port name in the board component matches the port where you have your board connected.</li>
      <li>Verify that the port name has quotation marks.</li>
      <li>ModelPlug will not connect to the board if there is another application using the port. Verify you do not have other applications using the port.</li>
      <li>If the simulation log does not show the board capabilities, try uploading the Firmata to your board again.</li>
      <li>When using multiple boards, your operating system may have changed the port names. Verify that the port&nbsp;names in the components correspond to each hardware board.</li>
      <li>Some boards like the Arduino Leonardo and compatible require the parameter UseDTR set to true. Change the parameter and test your board.</li>
      </ol>
      <p>If you are still having problems, you can go to&nbsp;<a href=\"http://community.wolfram.com/\">http://community.wolfram.com</a>&nbsp;and ask a question.</p>
      <h4>What Next?</h4>
      <p>OpenModelicaArduino contains a series of basic examples showing the functionality of the components. You can check the Examples under ModelPlug.Examples. Once you have learned how to use the basic components of OpenModelicaArduino, you can check the Arduino Playground to learn how to connect other sensors and actuators (<a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a>).</p>", revisions = ""), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end GettingStarted;

  package Pins "Components to access the board I/O"
    extends Internal.Icons.Block;

model AnalogInput "Reads an analog signal from the specified pin"
  extends OpenModelicaArduino.Internal.Icons.Block;
  Modelica.Blocks.Interfaces.RealOutput y annotation(Placement(visible = true, transformation(origin = {110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  parameter Integer Pin = 0 "Number of the analog pin";
  OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(Placement(visible = true, transformation(origin = {-100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  parameter Real InitValue = 0 "Initial value until the board responds" annotation(Dialog(group = "Initialization"));
  parameter Real MinValue = 0 "Minimum value when the ADC reads 0" annotation(Dialog(group = "Scaling"));
  parameter Real MaxValue = 1 "Maximum value when the ADC reads 1024" annotation(Dialog(group = "Scaling"));
equation
  y = OpenModelicaArduino.Internal.ExternalFunctions.readAnalogPin(Pin, MinValue, MaxValue, InitValue, pinConnector);
  annotation(Documentation(info = "<p>Reads an analog signal from the specified pin. This component uses the 'analogRead' function of Arduino.</p>
 <p><strong>Signal Range:</strong> By default, the signal goes from 0 to 1 where 0 represents no voltage and 1 the voltage reference of the ADC in the board. This signal can be scaled by setting the 'MinValue' and 'MaxValue' parameters.</p>
 <p>Not all pins support analog input. Check the documentation of your board to find the pin capabilities.</p>
 <p>&nbsp;</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10},extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20},extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}));
end AnalogInput;













model AnalogOutput "Writes an analog signal to the specified pin"
  extends OpenModelicaArduino.Internal.Icons.Block;
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-110, -0}, extent = {{-25, -25}, {25, 25}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  parameter Integer Pin = 0 "Number of the PWM/DAC pin";
  OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(Placement(visible = true, transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  parameter Real MinValue = 0 "Value considered as minimum by the PWM/DAC" annotation(Dialog(group = "Scaling"));
  parameter Real MaxValue = 1 "Value considered as maximum by the PWM/DAC" annotation(Dialog(group = "Scaling"));
protected
  Real scaled_u = (u - MinValue) / (MaxValue - MinValue);
equation
  OpenModelicaArduino.Internal.ExternalFunctions.writeAnalogPin(Pin, pinConnector, scaled_u);
  annotation(Documentation(info = "<p>Writes an analog signal to the specified pin. This component uses the 'analogWrite' function of Arduino.</p>
 <p><strong>Signal Range:</strong> By default, the signal goes from 0 to 1, where 0 represents no voltage and 1 the maximum voltage that PWM/DAC of your board can provide. This signal can be scaled by setting the 'MinValue' and 'MaxValue' parameters.</p>
 <p>Analog outputs use the PWM capabilities of the pins, therefore, they do not provide a continuous signal. If you want to get a continuous signal, you need to add a lowpass filter. Check the Arduino Playground (<a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a>) for more information on PWM.</p>
 <p>Not all pins support analog output. Check the documentation of your board to find the pin capabilities.</p>
 <p>&nbsp;</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10},extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Output", textStyle = {TextStyle.Bold})}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20},extent = {{-75, -15}, {75, 25}}, textString = "Analog", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Output", textStyle = {TextStyle.Bold})}));
end AnalogOutput;
    
    




    model DigitalInput "Reads a digital signal from the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.BooleanOutput y annotation(Placement(visible = true, transformation(origin = {110, 1.643}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {101.75, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Number of the digital pin";
      parameter Boolean InitValue = false "Initial value until the board responds" annotation(Dialog(group = "Initialization"));
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      y = OpenModelicaArduino.Internal.ExternalFunctions.readDigitalPin(Pin, InitValue, pinConnector);
      annotation(Documentation(info = "<p>Reads a digital signal from&nbsp;the specified pin. This component uses the 'digitalRead' function of Arduino.</p>
      <p><strong>Signal Range:</strong>&nbsp;This component accepts only Boolean signals (true/false).</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10}, extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20},extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}));
    end DigitalInput;




    model DigitalOutput "Writes a digital signal to the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.BooleanInput u annotation(Placement(visible = true, transformation(origin = {-110, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Number of the digital pin";
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(Placement(visible = true, transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      OpenModelicaArduino.Internal.ExternalFunctions.writeDigitalPin(Pin, pinConnector, u);
      annotation(Documentation(info = "<p>Writes a digital signal to the specified pin. This component uses the 'digitalWrite' function of Arduino.</p>
     <p><strong>Signal Range:</strong>&nbsp;This component accepts only Boolean signals (true/false).</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(origin = {0, 10},extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Output", textStyle = {TextStyle.Bold})}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {0, 170, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(origin = {0, 20},extent = {{-75, -15}, {75, 25}}, textString = "Digital", textStyle = {TextStyle.Bold}), Text(origin = {0, -20}, extent = {{-75, -15}, {75, 25}}, textString = "Input", textStyle = {TextStyle.Bold})}));
    end DigitalOutput;




    model Servo "Controls a servo motor attached to the specified pin"
      extends OpenModelicaArduino.Internal.Icons.Block;
      Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Internal.Interfaces.PinConnector pinConnector annotation(Placement(visible = true, transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      parameter Integer Pin = 0 "Pin number of the servo";
      parameter OpenModelicaArduino.Internal.Types.ServoUnit InputUnit "if None (Default) the servo receives a signal between 0 and 1. If Degrees the control signal is from 0 to 180. If Radians the signal is from 0 to Pi.";
      parameter Integer MinPulse = 544 "The pulse width, in microseconds, corresponding to the minimum (0-degree) angle on the servo." annotation(Dialog(group = "Advanced"));
      parameter Integer MaxPulse = 2400 "The pulse width, in microseconds, corresponding to the maximum (180-degree) angle on the servo." annotation(Dialog(group = "Advanced"));
      Real outputSignal;
    equation
      outputSignal = if InputUnit == OpenModelicaArduino.Internal.Types.ServoUnit.None then u else if InputUnit == OpenModelicaArduino.Internal.Types.ServoUnit.Degrees then u / 180 else if InputUnit == OpenModelicaArduino.Internal.Types.ServoUnit.Radians then u / Modelica.Constants.pi else u;
      OpenModelicaArduino.Internal.ExternalFunctions.writeServoPin(Pin, pinConnector, u, MinPulse, MaxPulse);
      annotation(Documentation(info = "<p>Controls a servo motor attached to the specified pin. This component uses the 'Servo' library of Arduino.</p>
      <p><strong>Signal Range:</strong> By default, the range goes from 0 to 1, which corresponds to 0 to 180 degrees. If you want to input values in radians, you can change the parameter 'InputUnit' from 'Degrees' to 'Radians'.</p>
      <p>If your servo does not work correctly with the default settings, you can set the parameters 'MinPulse' and 'MaxPulse'. To get more information on how to configure a servo, you can check the documentation of the Servo library,&nbsp;<a href=\"http://arduino.cc/en/reference/servo\">http://arduino.cc/en/reference/servo</a>.</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -85}, {100, 85}}, radius = 40), Rectangle(fillColor = {0,  255, 127}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -60}, {90, 60}}, radius = 40), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "Pin %Pin"), Text(extent = {{-75, -25}, {75, 25}}, textString = "Servo", textStyle = {TextStyle.Bold})}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 40), Rectangle(fillColor = {0,  255, 127}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}, radius = 40), Text(extent = {{-75, -25}, {75, 25}}, textString = "Servo", textStyle = {TextStyle.Bold})}));
    end Servo;

    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {-30, 30}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50), Rectangle(visible = true, origin = {30, 30}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50), Rectangle(visible = true, origin = {-30, -30}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50), Rectangle(visible = true, origin = {30, -30}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-25, -25}, {25, 25}}, radius = 50)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end Pins;

  package Boards "Components to connect to the Firmata boards"
    model Arduino "Component with default configuration for Arduino boards"
      parameter Internal.Types.SerialPort Port = "COM1" "Name of the serial port";
      parameter Boolean ShowPinCapabilities = true "Set to true if you want to see the capabilities of the pins in your board";
      parameter Boolean UseDTR = false "Some boards like Arduino Leonardo require UseDTR=true";
      OpenModelicaArduino.Internal.ExternalFunctions.FirmataBoardObject board = Internal.ExternalFunctions.FirmataBoardObject.constructor(Port, ShowPinCapabilities, integer(UpdatePeriod * 1000), BaudRate, UseDTR);
      OpenModelicaArduino.Internal.Interfaces.BoardConnector boardConnector annotation(Placement(visible = true, transformation(origin = {-90, -12.256}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-100, -100}, {100, 100}}, rotation = 0)));
    protected
      discrete Integer BoardId(start = -1) "Board Identifier. Initialized automatically keep as -1.";
      constant Modelica.SIunits.Time UpdatePeriod = 0.01 "Sampling interval. The standard Firmata has a maximum samplig period of 10 ms (0.01 s)";
      constant Integer BaudRate = 57600 "Baud rate used to comunicate with the board";
    equation
      when initial() then
        BoardId = Internal.ExternalFunctions.getBoardId(board);
      end when;
      when sample(0, UpdatePeriod) then
        OpenModelicaArduino.Internal.ExternalFunctions.updateBoard(BoardId);
      end when;
      boardConnector = BoardId;
      annotation(Documentation(info = "<p>This component provides a ready-to-use configuration for the Arduino board. This component has been tested with Arduino Uno, Arduino Mega 2560, and Arduino Mini, but it should work with any Arduino board.</p>
  <p>When running the simulation, you will get a list of the capabilities of every pin. If you want to stop showing the capabilities every time you simulate, you need to set the property 'ShowCapabilities' to false. If you are not sure about the location of a specific pin in your board, you should&nbsp;check the documentation for&nbsp;your board.</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.562, 9.352}, fillColor = {42, 85, 94}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Ellipse(origin = {-8.451, 32.225}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}, endAngle = 360), Ellipse(origin = {21.549, 32.225}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}, endAngle = 360), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "%Port"), Text(origin = {5, -30}, lineColor = {255, 255, 255}, extent = {{-75, 30}, {75, -30}}, textString = "Arduino")}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.562, 9.352}, fillColor = {42, 85, 94}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Ellipse(origin = {-8.451, 32.225}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}, endAngle = 360), Ellipse(origin = {21.55, 32.22}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.23}, {15, 12.23}}, endAngle = 360), Text(origin = {-5, -15}, lineColor = {255, 255, 255}, extent = {{-75, 25}, {95, -35}}, textString = "Arduino", textStyle = {TextStyle.Bold})}));
    end Arduino;

    model StandardFirmata "Component with default configuration for any board with the standard Firmata"
      parameter Internal.Types.SerialPort Port = "COM1" "Name of the serial port";
      parameter Boolean ShowPinCapabilities = true "Set to true if you want to see the capabilities of the pins in your board";
      parameter Boolean UseDTR = false "Set to true if your board requires DTR";
      OpenModelicaArduino.Internal.Interfaces.BoardConnector boardConnector annotation(Placement(visible = true, transformation(origin = {-90, -14.401}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-100, -100}, {100, 100}}, rotation = 0)));
      OpenModelicaArduino.Internal.ExternalFunctions.FirmataBoardObject board = Internal.ExternalFunctions.FirmataBoardObject.constructor(Port, ShowPinCapabilities, integer(UpdatePeriod * 1000), BaudRate, UseDTR);
    protected
      discrete Integer BoardId(start = -1) "Board Identifier. Initialized automatically keep as -1.";
      constant Modelica.SIunits.Time UpdatePeriod = 0.01 "Sampling interval. The standard Firmata has a maximum samplig period of 10 ms (0.01 s)";
      constant Integer BaudRate = 57600 "Baud rate used to comunicate with the board";
    equation
      when initial() then
        BoardId = Internal.ExternalFunctions.getBoardId(board);
      end when;
      when sample(0, UpdatePeriod) then
        OpenModelicaArduino.Internal.ExternalFunctions.updateBoard(BoardId);
      end when;
      boardConnector = BoardId;
      annotation(Documentation(info = "<p>This component provides a ready-to-use configuration for boards flashed with the StandardFirmata. This component has been tested with chipKIT UNO and Teensy 3.1, but it should work with any board supporting the StandardFirmata.</p>
      <p>When running the simulation, you will get a list of the capabilities of every pin. If you want to stop showing the capabilities every time you simulate, you need to set the property 'ShowCapabilities' to False. If you are not sure about the location of an specific pin in your board, you should check the documentation for your board.</p>
  <p>&nbsp;</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.56, 9.35}, fillColor = {0, 170, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "%Port"), Text(origin = {5, -10}, lineColor = {255, 255, 255}, extent = {{-75, 30}, {75, -30}}, textString = "Standard")}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.56, 9.35}, fillColor = {0, 170, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10),  Text(origin = {-5, -15}, lineColor = {255, 255, 255}, extent = {{-75, 25}, {95, -35}}, textString = "Standard", textStyle = {TextStyle.Bold})}));
    end StandardFirmata;


    model CustomFirmata "Component that allows custom configuration of the Firmata"
      parameter Internal.Types.SerialPort Port = "COM1" "Name of the serial port";
      parameter Boolean ShowPinCapabilities = true "Set to true if you want to see the capabilities of the pins in your board";
      parameter Boolean UseDTR = false "Some boards like Arduino Leonardo require UseDTR=true";
      OpenModelicaArduino.Internal.Interfaces.BoardConnector boardConnector annotation(Placement(visible = true, transformation(origin = {-90, -14.401}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0,0}, extent = {{-100, -100}, {100, 100}}, rotation = 0)));
      OpenModelicaArduino.Internal.ExternalFunctions.FirmataBoardObject board = Internal.ExternalFunctions.FirmataBoardObject.constructor(Port, ShowPinCapabilities, integer(UpdatePeriod * 1000), BaudRate, UseDTR);
      parameter Modelica.SIunits.Time UpdatePeriod = 0.01 "Sampling interval. The standard Firmata has a maximum samplig period of 10 ms (0.01 s)";
      parameter Integer BaudRate = 57600 "Baud rate used to communicate with the board";
    protected
      discrete Integer BoardId(start = -1) "Board Identifier. Initialized automatically keep as -1.";
    equation
      when initial() then
        BoardId = Internal.ExternalFunctions.getBoardId(board);
      end when;
      when sample(0, UpdatePeriod) then
        OpenModelicaArduino.Internal.ExternalFunctions.updateBoard(BoardId);
      end when;
      boardConnector = BoardId;
      annotation(Documentation(info = "<p>This component is for advanced users who&nbsp;can modify the Firmata source code or have a non-Arduino board running Firmata.&nbsp;This component allows you to specify your preferred baud rate and sampling interval.&nbsp;</p>
      <p>The baud rate is specified with the 'BaudRate' property. You have to consider that standard serial ports do not support arbitrary baud rates.</p>
      <p>When setting the 'UpdatePeriod' property, it is necessary to change the \"Interval length\" in the simulation settings, to match the update period.</p>", revisions = ""), Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.56, 9.35}, fillColor = {170, 0, 127}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "%Port"), Text(origin = {5, -10}, lineColor = {255, 255, 255}, extent = {{-75, 30}, {75, -30}}, textString = "Custom")}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.56, 9.35}, fillColor = {170, 0, 127}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10),  Text(origin = {-5, -15}, lineColor = {255, 255, 255}, extent = {{-75, 25}, {95, -35}}, textString = "Custom", textStyle = {TextStyle.Bold})}));
    end CustomFirmata;


    model ArduinoLeonardo "Component with configuration for Arduino Leonardo boards"
      parameter Internal.Types.SerialPort Port = "COM1" "Name of the serial port";
      parameter Boolean ShowPinCapabilities = true "Set to true if you want to see the capabilities of the pins in your board";
      parameter Boolean UseDTR = true "Some boards like Arduino Leonardo require UseDTR=true";
      OpenModelicaArduino.Internal.ExternalFunctions.FirmataBoardObject board = Internal.ExternalFunctions.FirmataBoardObject.constructor(Port, ShowPinCapabilities, integer(UpdatePeriod * 1000), BaudRate, UseDTR);
      OpenModelicaArduino.Internal.Interfaces.BoardConnector boardConnector annotation(Placement(visible = true, transformation(origin = {-90, -12.256}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -0}, extent = {{-100, -100}, {100, 100}}, rotation = 0)));
    protected
      discrete Integer BoardId(start = -1) "Board Identifier. Initialized automatically keep as -1.";
      constant Modelica.SIunits.Time UpdatePeriod = 0.01 "Sampling interval. The standard Firmata has a maximum samplig period of 10 ms (0.01 s)";
      constant Integer BaudRate = 57600 "Baud rate used to comunicate with the board";
    equation
      when initial() then
        BoardId = Internal.ExternalFunctions.getBoardId(board);
      end when;
      when sample(0, UpdatePeriod) then
        OpenModelicaArduino.Internal.ExternalFunctions.updateBoard(BoardId);
      end when;
      boardConnector = BoardId;
      annotation(Icon(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.562, 9.352}, fillColor = {42, 85, 94}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Ellipse(origin = {-8.451, 32.225}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}, endAngle = 360), Ellipse(origin = {21.549, 32.225}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}, endAngle = 360), Text(origin = {0, -130}, extent = {{-100, -20}, {100, 20}}, textString = "%Port"), Text(origin = {5, -30}, lineColor = {255, 255, 255}, extent = {{-75, 30}, {75, -30}}, textString = "Leonardo")}), Diagram(coordinateSystem( initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(origin = {10.562, 9.352}, fillColor = {42, 85, 94}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}, {-85.751, 70.648}}, smooth = Smooth.Bezier), Rectangle(origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Ellipse(origin = {-8.451, 32.225}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}, endAngle = 360), Ellipse(origin = {21.55, 32.22}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.23}, {15, 12.23}}, endAngle = 360), Text(origin = {-5, -15}, lineColor = {255, 255, 255}, extent = {{-75, 25}, {95, -35}}, textString = "Leonardo", textStyle = {TextStyle.Bold})}), Documentation(info = "<p>This component provides a ready-to-use configuration for the Arduino Leonardo board. This component works for any board using native USB communication, like the Arduino Esplora.</p>
     <p>When running the simulation, you will get a list of the capabilities of every pin. If you want to stop showing the capabilities every time you simulate, you need to set the property &quot;ShowCapabilities&quot; to false.</p>", revisions = ""), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {10.751, 9.352}, fillColor = {42, 85, 94}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.751, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.475, -85.465}, {-85.751, -89.352}, {74.249, -89.352}, {81.746, -83.29}, {84.249, -76.041}, {84.249, 34.141}, {79.249, 43.202}, {54.249, 43.202}, {52.314, 67.486}, {31.729, 70.648}}, smooth = Smooth.Bezier), Rectangle(visible = true, origin = {-71.211, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(visible = true, origin = {-79.366, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Rectangle(visible = true, origin = {0.725, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-54.177, -6.186}, {54.177, 6.186}}, radius = 10), Rectangle(visible = true, origin = {22.372, -63.814}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-64.177, -6.186}, {64.177, 6.186}}, radius = 10), Rectangle(visible = true, origin = {25.52, -30}, fillPattern = FillPattern.Solid, extent = {{-51.029, -13.424}, {51.029, 13.424}}, radius = 10), Rectangle(visible = true, origin = {76.549, -63.624}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {66.549, -63.624}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {56.549, -63.624}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {46.549, -63.624}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {36.549, -63.624}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {26.549, -63.624}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {46.549, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {36.549, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {26.549, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {16.549, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {6.549, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-3.451, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-13.451, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-23.451, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-33.451, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-43.451, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-33.451, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-23.451, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-13.451, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-3.451, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Ellipse(visible = true, origin = {-8.451, 43.801}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}), Ellipse(visible = true, origin = {21.549, 43.801}, lineColor = {255, 255, 255}, fillColor = {42, 85, 94}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-15, -12.225}, {15, 12.225}}), Text(visible = true, origin = {10, 23.151},  extent = {{-40, -10}, {40, 10}}, textString = "Leonardo")}));
    end ArduinoLeonardo;

    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {12.096, 9.352}, fillColor = {0, 128, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-85.75100000000001, 70.648}, {-102.737, 66.761}, {-105.751, 50.648}, {-105.751, -69.352}, {-99.47499999999999, -85.465}, {-85.75100000000001, -89.352}, {74.249, -89.352}, {81.746, -83.29000000000001}, {86.32899999999999, -74.352}, {84.249, 34.141}, {82.185, 50.648}, {74.732, 67.486}, {52.373, 70.648}, {31.729, 70.648}}, smooth = Smooth.Bezier), Rectangle(visible = true, origin = {-69.67700000000001, 40}, fillColor = {106, 108, 116}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-32.24, -20}, {32.24, 20}}, radius = 10), Rectangle(visible = true, origin = {-77.83199999999999, -45.041}, fillPattern = FillPattern.Solid, extent = {{-25.915, -15.041}, {25.915, 15.041}}, radius = 10), Rectangle(visible = true, origin = {24.177, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-54.177, -6.186}, {54.177, 6.186}}, radius = 10), Rectangle(visible = true, origin = {23.906, -63.814}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-64.17700000000001, -6.186}, {64.17700000000001, 6.186}}, radius = 10), Rectangle(visible = true, origin = {21.91, 0}, fillPattern = FillPattern.Solid, extent = {{-28.09, -25}, {28.09, 25}}, radius = 10), Rectangle(visible = true, origin = {78.083, -63.624}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {68.083, -63.624}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {58.083, -63.624}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {48.083, -63.624}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {38.083, -63.624}, fillColor = {250, 105, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {28.083, -63.624}, fillColor = {243, 134, 48}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {70.00100000000001, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {60.001, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {50.001, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {40.001, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {30.001, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {20.001, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {10.001, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {0.001, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-9.999000000000001, 70}, fillColor = {167, 219, 216}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-19.999, 70}, fillColor = {105, 210, 231}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-31.917, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-21.917, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-11.917, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50), Rectangle(visible = true, origin = {-1.917, -63.624}, fillColor = {204, 208, 224}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, borderPattern = BorderPattern.Engraved, extent = {{-4.118, -3.624}, {4.118, 3.624}}, radius = 50)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end Boards;

  package Examples "A collection of examples to help you get started"
    extends Modelica.Icons.ExamplesPackage;

model BlinkLed "Basic example of blinking an LED"
  extends Modelica.Icons.Example;
  replaceable OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0", ShowPinCapabilities = true, UseDTR = false) annotation(Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpenModelicaArduino.Pins.DigitalOutput digitalOutput(Pin = 9) annotation(Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 1, startTime = 0, width = 50) annotation(Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
    Placement(visible = true, transformation(origin = {35, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(digitalOutput.pinConnector, arduino.boardConnector) annotation(Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, -0}}, color = {72, 73, 79}));
  connect(booleanPulse.y, digitalOutput.u) annotation(Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, 0}, {4.5, 0}}, color = {255, 0, 255}));
  annotation(experiment(StopTime = 10, Interval = 0.001), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "diagram", Documentation(info = "<html><h4>Hardware Components&nbsp;Used&nbsp;</h4>
<ul>
<li>1 Arduino board</li>
<li>1 LED (optional)</li>
<li>1 resistor 680 ohms (optional)</li>
</ul>
<h4>Description</h4>
<p>This example uses the DigitalOutput component to control the LED attached to the Arduino board on pin 13. It uses a BooleanPulse from the Modelica library to produce an On/Off signal that is fed into the DigitalOutput component. This will make the LED attached to the pin blink.</p>
<p>You can go ahead and add more LEDs to the board as shown in the following figure. This will require you to add one more DigitalOutput component to control the LED on pin 9.</p>
<p><img src=\"Modelica://Desktop/Modelica/OpenModelicaArduino/Resources/Images/Blink.png\" alt=\"\" /></p></html>", revisions = ""), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5}), graphics = {Text(origin = {0, 30}, extent = {{-50, -10}, {50, 10}}, textString = "Blinking an LED", fontSize = 24)}));
end BlinkLed;














    model DimmingLed "Changing the intensity of an LED"
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0", ShowPinCapabilities = true) annotation(Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Pins.AnalogOutput analogOutput(Pin = 9) annotation(Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Sine sine(amplitude = 1 / 2, freqHz = 1 / 4, offset = 1 / 2) annotation(Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {35, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sine.y, analogOutput.u) annotation(Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, 0}, {4.5, 0}}, color = {0, 0, 127}));
      connect(analogOutput.pinConnector, arduino.boardConnector) annotation(Line(visible = true, origin = {20, -10}, points = {{-10, -0}, {10, 0}}));
      annotation(experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "diagram", Documentation(info = "<html><h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <ul>
    <li>1 Arduino board</li>
    <li>1 LED</li>
    <li>1 resistor 680 ohms</li>
    </ul>
    <h4><br />Description</h4>
    <p>This example uses the AnalogOutput component to change the light intensity of an LED. AnalogOutput uses the Arduino function 'analogWrite', which produces a PWM (Pulse-Width Modulated) signal. This type of signal can be used to directly&nbsp;control the LED intensity. The following figure shows the connections.</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/Dimming.png\" alt=\"\" /></p>
    <p>You can check the Arduino Playground to know more about PWM outputs.&nbsp;</p></html>", revisions = ""), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, origin = {0, 30}, extent = {{-50.857, -10}, {50.857, 10}}, textString = "Dimming LED", fontSize = 24)}));
    end DimmingLed;


    model ReadSensor "Reading analog signals"
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0") annotation(Placement(visible = true, transformation(origin = {0, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput(Pin = 10) annotation(Placement(visible = true, transformation(origin = {-30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(Pin = 16) annotation(Placement(visible = true, transformation(origin = {30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Greater greater1 annotation(Placement(visible = true, transformation(origin = {-10, -15}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Reference(k = 0.5) annotation(Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(greater1.y, digitalOutput.u) annotation(Line(visible = true, origin = {-38.744, -2.5}, points = {{17.744, -12.5}, {-6.256, -12.5}, {-6.256, 17.5}, {-1.256, 17.5}}, color = {255, 0, 255}));
      connect(analogInput1.y, greater1.u1) annotation(Line(visible = true, origin = {33.994, -5}, points = {{6.006, 20}, {11.006, 20}, {11.006, 0}, {-23.994, 0}, {-23.994, -10}, {-31.994, -10}}, color = {0, 0, 127}));
      connect(analogInput1.pinConnector, arduino.boardConnector) annotation(Line(visible = true, origin = {10, 15}, points = {{10, 0}, {-10, 0}}, color = {72, 73, 79}));
      connect(digitalOutput.pinConnector, arduino.boardConnector) annotation(Line(visible = true, origin = {-10, 15}, points = {{-10, 0}, {10, 0}}, color = {72, 73, 79}));
      connect(Reference.y, greater1.u2) annotation(Line(visible = true, origin = {12.75, -29}, points = {{6.25, -1}, {-2.75, -1}, {-2.75, 6}, {-10.75, 6}}, color = {0, 0, 127}));
      annotation(experiment(StartTime = 0.0, StopTime = 10.0, Interval = 0.001, __Wolfram_Algorithm = "dassl", Tolerance = 1e-006, __Wolfram_SynchronizeWithRealTime = true), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "diagram", Documentation(info = "<html><h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <ul>
    <li>1 Arduino board</li>
    <li>1 LED</li>
    <li>1 resistor 680 ohms</li>
    <li>1 potentiometer 100 K</li>
    </ul>
    <h4>Description</h4>
    <p>This example shows how to read an analog voltage using the AnalogInput component. The analog signal is then compared to a reference, and if the signal is above the reference, it will turn on an LED. You can see in the following figure how to build this example.</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/ReadAnalog.png\" alt=\"\" /></p>
    <p>You can see that pin A0 for the Arduino corresponds to pin number 14 for the Firmata. For other boards, the pin numbering may vary.</p>
    <p>Run the simulation and move the potentiometer. You should see that when the position of the shaft is near the middle, the LED changes state.</p>
    <p>The AnalogInput component returns a signal between 0 and 1. This value represents the voltage between 0 and the reference voltage. If you prefer to get the signal directly in volts, you need to change the 'MaxValue' property to the reference voltage, but generally it is either 5 V or 3.3 V.</p></html>", revisions = ""), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, origin = {0, 35}, extent = {{-50, -10}, {50, 10}}, textString = "Reading a Sensor", fontSize = 24)}));
    end ReadSensor;


    model MoveServo "Using servos"
      extends Modelica.Icons.Example;
      Boards.Arduino arduino(Port = "COM1") annotation(Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Pins.Servo servo(Pin = 10) annotation(Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.ExpSine expSine(offset = 0.5, amplitude = 0.5, freqHz = 0.4, damping = 0.1) annotation(Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(expSine.y, servo.u) annotation(Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, -0}, {4.5, 0}}, color = {0, 0, 127}));
      connect(servo.pinConnector, arduino.boardConnector) annotation(Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, -0}}, color = {72, 73, 79}));
      annotation(experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "info", Documentation(info = "<h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <ul>
    <li>1 Arduino</li>
    <li>1 5 V servo</li>
    <li>1 external 5 V power source</li>
    </ul>
    <h4>Description</h4>
    <p>This example shows how to control a servo by using the Servo component. You can find the diagram in&nbsp;the following figure.</p>
    <p>For this example, it is recommended to use an external power source to provide voltage for the servo. This is because the power from the Arduino may not be enough to supply the servo. If you are not sure how to connect your servo, take a look at the reference in the Arduino Playground (<a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a>).</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/ServoExample.png\" alt=\"\" /></p>
    <p>Servos are controlled with a signal in the range of 0 to 1, where 0 corresponds to 0 degrees of rotation and 1 to 180 degrees. This example makes the servo bounce from 0 to 180 degrees until it gets stable around 90 degrees.</p>", revisions = ""), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, origin = {-0, 30}, extent = {{-50, -10}, {50, 10}}, textString = "Controlling a Servo", fontSize = 24)}));
    end MoveServo;

    model SimpleONOFF "A simple On/Off controller"
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0")  annotation(Placement(visible = true, transformation(origin = {0, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1( MaxValue = 3.3 * 100,Pin = 16) annotation(Placement(visible = true, transformation(origin = {30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Reference(k = 40) annotation(Placement(visible = true, transformation(origin = {35, -25}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
      Modelica.Blocks.Math.Add add1(k2 = +1, k1 = -1) annotation(Placement(visible = true, transformation(origin = {-0, -25}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Pins.DigitalOutput digitalOutput(Pin = 10) annotation(Placement(visible = true, transformation(origin = {-30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Hysteresis hysteresis(uLow = -1, uHigh = 1) annotation(Placement(visible = true, transformation(origin = {-30, -25}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {30, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(hysteresis.u, add1.y) annotation(Line(visible = true, origin = {-14.5, -25}, points = {{-3.5, -0}, {3.5, 0}}, color = {0, 0, 127}));
      connect(digitalOutput.u, hysteresis.y) annotation(Line(visible = true, origin = {-43.25, 5}, points = {{3.25, 10}, {-1.75, 10}, {-1.75, -30}, {2.25, -30}}, color = {255, 0, 255}));
      connect(digitalOutput.pinConnector, arduino.boardConnector) annotation(Line(visible = true, origin = {-10, 15}, points = {{-10, 0}, {10, 0}}));
      connect(add1.u2, Reference.y) annotation(Line(visible = true, origin = {18.5, -23}, points = {{-6.5, -8}, {1.5, -8}, {1.5, -2}, {5.5, -2}}, color = {0, 0, 127}));
      connect(analogInput1.y, add1.u1) annotation(Line(visible = true, origin = {33.506, 12}, points = {{6.494, 3}, {11.494, 3}, {11.494, -17}, {-13.506, -17}, {-13.506, -31}, {-21.506, -31}}, color = {0, 0, 127}));
      connect(arduino.boardConnector, analogInput1.pinConnector) annotation(Line(visible = true, origin = {10, 15}, points = {{-10, 0}, {10, 0}}));
      annotation(experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "diagram", Documentation(info = "<html><h4>Hardware&nbsp;Components Used</h4>
    <ul>
    <li>1 Arduino</li>
    <li>1 LM35 temperature sensor</li>
    <li>1 2N2222 transistor</li>
    <li>1 1N4001 diode</li>
    <li>1 resistor 1 kOhm</li>
    <li>1 relay 5 V</li>
    <li>1 external 5 V power source</li>
    <li>1 heater or fan</li>
    </ul>
    <h4>Description</h4>
    <p>This example is a simple ON/OFF controller and can be used for&nbsp;either heating or cooling. It uses an LM35 to read the temperature, and based on that temperature, the controller switches a relay on or off. You can attach a fan or a heater&nbsp;to the relay, depending on the operation you want to perform. You can find the diagram in the following figure. <br /><strong>Note:</strong>&nbsp;You need to be careful when using relays that control electrical equipment using AC voltage, because an incorrect connection may damage your board.</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/SimpleONOFF.png\" alt=\"\" /></p>
    <p>The target temperature is set by a constant component. The measured temperature is subtracted from the reference in order to obtain the error. The error signal is fed into the hysteresis component, which will send a Boolean signal to control the relay. If you want cooling instead of heating, you need to invert the logic of this signal.</p></html>"), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, origin = {-0, 35}, extent = {{-50, -10}, {50, 10}}, textString = "Simple ON/OFF Control", fontSize = 24)}));
    end SimpleONOFF;


    model UsingArduinoLeonardo "Basic example of blinking an LED"
      extends OpenModelicaArduino.Examples.BlinkLed(redeclare ModelPlug.Boards.ArduinoLeonardo arduino);
      annotation(experiment(StopTime = 10, Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true), Documentation(info = "<h4>Hardware Components&nbsp;Used&nbsp;</h4>
   <ul>
   <li>1 Arduino Leonardo board</li>
   <li>1 LED (optional)</li>
   <li>1 resistor 680 ohms (optional)</li>
   </ul>
   <h4>Description</h4>
   <p>This example shows how the 
   <a href=\\\"modelica://OpenModelicaArduino.Examples.BlinkLed\\\">BlinkLed</a> can be used with an Arduino Leonardo.</p>", revisions = ""));
    end UsingArduinoLeonardo;

    model UsingCustomBoard "Using a Firmata-compatible board"
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 1) annotation(Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Pins.DigitalOutput digitalOutput(Pin = 13) annotation(Placement(visible = true, transformation(origin = {-0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Boards.CustomFirmata customFirmata(Port = "COM1", BaudRate = 115200, UpdatePeriod = 0.001) annotation(Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(digitalOutput.pinConnector, customFirmata.boardConnector) annotation(Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, -0}}));
      connect(booleanPulse.y, digitalOutput.u) annotation(Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, 0}, {4.5, 0}}, color = {255, 0, 255}));
      annotation(experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "info", Documentation(info = "<h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <p>- 1 Teensy 3.1 board</p>
    <h4>Description</h4>
    <p>This example shows how to use a board with a custom version of Firmata. The Teensy board works perfectly with the StandardFirmata, but in this example it is modified in order get a faster data transfer speed.</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/CustomExample.png\" alt=\"\" /></p>
    <p><br />The main difference of the CustomFirmata component is that it allows you to set the baud rate to the sampling interval that you want to use. In this case the oce of the StandardFirmata is modified to use a baud rate of 115200. You can perform this modification in the source code. Search for the line:</p>
    <p><span style=\"font-family: 'courier new', courier;\">Firmata.Begin(57600);</span></p>
    <p>and changing it to</p>
    <p><span style=\"font-family: 'courier new', courier;\">Firmata.Begin(115200);</span></p>
    <p>This initializes the serial port at the given speed. The next change that you can make is reducing the minimum sampling interval. You can find this tweak by searching in the code the text:</p>
    <p><span style=\"font-family: 'courier new', courier;\">#define MINIMUM_SAMPLING_INTERVAL 10</span></p>
    <p>and changing it to</p>
    <p><span style=\"font-family: 'courier new', courier;\">#define MINIMUM_SAMPLING_INTERVAL 1</span></p>
    <p>This changes the sampling interval limit from 10 ms to 1ms. You have to consider that this change is possible because the Teensy board can run up to 96 MHz. If you change the sampling interval, it is necessary to change the simulation setting in SimulationCenter to 1 ms, as shown in the following figure.</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/Interval.png\" alt=\"\" /></p>", revisions = ""), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, origin = {0, 30}, extent = {{-50, -10}, {50, 10}}, textString = "Using a Custom Board", fontSize = 24)}));
    end UsingCustomBoard;

    model UsingStandardFirmata "Using a standard Firmata board"
      extends Modelica.Icons.Example;
      Boards.StandardFirmata standardFirmata(Port = "COM1", ShowPinCapabilities = true) annotation(Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput(Pin = 13) annotation(Placement(visible = true, transformation(origin = {-0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 1) annotation(Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(digitalOutput.pinConnector, standardFirmata.boardConnector) annotation(Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, 0}}, color = {72, 73, 79}));
      connect(booleanPulse.y, digitalOutput.u) annotation(Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, 0}, {4.5, 0}}, color = {255, 0, 255}));
      annotation(experiment(StopTime = 10, Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})), preferredView = "info", Documentation(info = "<h4>Hardware Components&nbsp;Used</h4>
    <ul>
    <li>Any board with standard Firmata (this example uses the Teensy board)</li>
    <li>1 LED (optional)</li>
    <li>1 resistor 680 ohms (optional)</li>
    </ul>
    <h4>Description</h4>
    <p>This example uses the DigitalOutput component to control the LED attached to a standard Firmata board on pin 13. It uses a BooleanPulse from the Modelica library to produce an On/Off signal that is fed into the DigitalOutput component. This will make the LED attached to the pin blink.</p>
    <p>You can go ahead and add more LEDs to the board as shown in the following figure. This will require you to add one more DigitalOutput component to control the LED on pin 9.</p>
    <p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/StandardFirmata.png\" alt=\"\" /></p>", revisions = ""), Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, origin = {-0, 30}, extent = {{-50, -10}, {50, 10}}, textString = "Using Standard Firmata", fontSize = 24)}));
    end UsingStandardFirmata;
    annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end Examples;

  package Internal "Internal classes that should not be used directly by the user"
    extends Modelica.Icons.Package;

    package Icons "Icons used in ModelPlug"
      extends Modelica.Icons.IconsPackage;

      partial model Block "Icon for blocks"
        annotation(experiment(NumberOfIntervals = -1), preferredView = "icon", Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -80}, {100, 80}}, radius = 40), Rectangle(visible = true, origin = {5, -5}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-95, -75}, {95, 75}}, radius = 40), Rectangle(visible = true, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -70}, {90, 70}}, radius = 30)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
      end Block;
      annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end Icons;

    package ExternalFunctions "External functions used to communicate with the boards"
      extends Modelica.Icons.Package;

      function readAnalogPin
        input Integer pin;
        input Real min;
        input Real max;
        input Real init;
        input Integer board;
        output Real value;
      
        external "C" value = readAnalogPin(pin, min, max, init, board) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end readAnalogPin;

      class FirmataBoardObject
        extends ExternalObject;

        function constructor "Function that call the external constructor"
          input String port;
          input Boolean showCapabilities;
          input Integer samplingMs;
          input Integer BaudRate;
          input Boolean UseDTR;
          output FirmataBoardObject board;
        
          external "C" board = boardConstructor(port, showCapabilities, samplingMs, BaudRate, UseDTR) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
        end constructor;

        function destructor "Function to destroy the object"
          input FirmataBoardObject board;
        
          external "C" boardDestructor(board) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
        end destructor;
      end FirmataBoardObject;

      function getBoardId
        input FirmataBoardObject board;
        output Integer id;
      
        external "C" id = getBoardId(board) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end getBoardId;

      function readDigitalPin
        input Integer pin;
        input Boolean init;
        input Integer board;
        output Boolean value;
      
        external "C" value = readDigitalPin(pin, init, board) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end readDigitalPin;

      function writeAnalogPin
        input Integer pin;
        input Integer board;
        input Real value;
      
        external "C" writeAnalogPin(pin, board, value) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end writeAnalogPin;

      function writeDigitalPin
        input Integer pin;
        input Integer board;
        input Boolean value;
      
        external "C" writeDigitalPin(pin, board, value) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end writeDigitalPin;

      function updateBoard
        input Integer board;
      
        external "C" updateBoard(board) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end updateBoard;

      function writeServoPin
        input Integer pin;
        input Integer board;
        input Real value;
        input Integer MinPulse;
        input Integer MaxPulse;
      
        external "C" writeServoPin(pin, board, value, MinPulse, MaxPulse) annotation(Include = "#include \"modelPlugFirmata.h\"", Library = "modelPlugFirmata", IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include", LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end writeServoPin;
      annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {11.425, 9.596}, extent = {{-101.424, -59.596}, {78.57599999999999, 40.404}}, textString = "EF")}), Documentation(info = "", revisions = ""), Diagram(coordinateSystem(extent = {{-148.5, 105}, {148.5, -105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(visible = true, fillColor = {209, 209, 209}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {75, 75}}), Polygon(visible = true, fillColor = {236, 236, 236}, fillPattern = FillPattern.Solid, points = {{-100, 75}, {-75, 100}, {100, 100}, {75, 75}}), Polygon(visible = true, fillColor = {177, 177, 177}, fillPattern = FillPattern.Solid, points = {{75, -100}, {75, 75}, {100, 100}, {100, -75}}), Text(visible = true, extent = {{-95.95, -91.88}, {63.97, 71.52}}, textString = "C")}));
    end ExternalFunctions;

    package Types "Type and unit definitions"
      extends Modelica.Icons.TypesPackage;
      type ServoUnit = enumeration(None, Degrees, Radians);
      type SerialPort = String annotation(Dialog(__Wolfram_serialPortSelector=true));
      annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end Types;

    package Interfaces "Package with connectors and partial models"
      extends Modelica.Icons.InterfacesPackage;
      connector BoardConnector = output Integer annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, origin = {-0, 0.74}, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-10, -9.26}, {10, 9.26}})}), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, origin = {-0, 0.74}, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-10, -9.26}, {10, 9.26}})}));
      connector PinConnector = input Integer annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Ellipse(visible = true, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -70}, {70, 70}})}), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Ellipse(visible = true, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -70}, {70, 70}})}));
    end Interfaces;
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {1.383, -4.142}, rotation = 45, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-15, 93.333}, {-15, 68.333}, {0, 58.333}, {15, 68.333}, {15, 93.333}, {20, 93.333}, {25, 83.333}, {25, 58.333}, {10, 43.333}, {10, -41.667}, {25, -56.667}, {25, -76.667}, {10, -91.667}, {0, -91.667}, {0, -81.667}, {5, -81.667}, {15, -71.667}, {15, -61.667}, {5, -51.667}, {-5, -51.667}, {-15, -61.667}, {-15, -71.667}, {-5, -81.667}, {0, -81.667}, {0, -91.667}, {-10, -91.667}, {-25, -76.667}, {-25, -56.667}, {-10, -41.667}, {-10, 43.333}, {-25, 58.333}, {-25, 83.333}, {-20, 93.333}}), Polygon(visible = true, origin = {10.102, 5.218}, rotation = -45, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-15, 87.273}, {15, 87.273}, {20, 82.273}, {20, 27.273}, {10, 17.273}, {10, 7.273}, {20, 2.273}, {20, -2.727}, {5, -2.727}, {5, -77.727}, {10, -87.727}, {5, -112.727}, {-5, -112.727}, {-10, -87.727}, {-5, -77.727}, {-5, -2.727}, {-20, -2.727}, {-20, 2.273}, {-10, 7.273}, {-10, 17.273}, {-20, 27.273}, {-20, 82.273}})}));
  end Internal;
  annotation(preferredView = "info", Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Documentation(info = "<h4>What Is ModelPlug?</h4>
<p>ModelPlug is a library that allows you to connect your simulations with the real world.&nbsp;It uses an Arduino board (or compatible) to send analog and digital signals to physical devices and receive signals from them.</p>
<p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/ModelPlugDocumentation-1.png\" alt=\"\" /></p>
<h4>What Can You Do with It?</h4>
<ul>
<li>Interact with your model by using buttons, switches, knobs, etc.</li>
<li>Input sensor information, for example, about&nbsp;light, temperature, position, pressure, etc.</li>
<li>Use actuators like motors, servos, and relays.</li>
<li>Quickly prototype your system by using SystemModeler blocks.</li>
</ul>
<p>&nbsp;</p>
<p>With ModelPlug, you can combine simulation models and real hardware. For example, you can get data from your hardware, design a control, and test it in real time.</p>
<p>&nbsp;</p>
<p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/ModelPlugDocumentation-2.png\" alt=\"\" /></p>
<p>&nbsp;You can build a real control panel and use it to control your simulation models.&nbsp;</p>
<p><img src=\"Modelica://OpenModelicaArduino/Resources/Images/ModelPlugDocumentation-3.png\" alt=\"\" /></p>
<h4>Overview of the Components</h4>
<p>ModelPlug provides the following components:</p>
<table style=\"height: 212px;\" width=\"479\">
<tbody>
<tr>
<td><strong>&nbsp;</strong></td>
<td style=\"text-align: center;\">&nbsp;<span class=\"Apple-style-span\"><strong>Inputs</strong></span></td>
<td>&nbsp;</td>
</tr>
<tr>
<td>Analog&nbsp;input</td>
<td>Reads analog values from the pins</td>
<td style=\"text-align: center;\"><img src=\"Modelica://OpenModelicaArduino/Resources/Images/analogInput.png\" alt=\"\" />&nbsp;</td>
</tr>
<tr>
<td>Digital&nbsp;input</td>
<td>Reads digital values from the pins</td>
<td style=\"text-align: center;\">&nbsp;<img src=\"Modelica://OpenModelicaArduino/Resources/Images/digitalInput.png\" alt=\"\" /></td>
</tr>
<tr>
<td><strong>&nbsp;</strong></td>
<td style=\"text-align: center;\">&nbsp;<span class=\"Apple-style-span\"><strong>Output</strong></span></td>
<td>&nbsp;</td>
</tr>
<tr>
<td>Analog output</td>
<td>Writes analog values to the pins</td>
<td style=\"text-align: center;\">&nbsp;<img src=\"Modelica://OpenModelicaArduino/Resources/Images/analogOutput.png\" alt=\"\" /></td>
</tr>
<tr>
<td>Digital output</td>
<td>Writes digital values to the pins</td>
<td style=\"text-align: center;\">&nbsp;<img src=\"Modelica://OpenModelicaArduino/Resources/Images/digitalOutput.png\" alt=\"\" /></td>
</tr>
<tr>
<td>Servo control</td>
<td>Writes the angle to servo motors</td>
<td>&nbsp;<img style=\"display: block; margin-left: auto; margin-right: auto;\" src=\"Modelica://OpenModelicaArduino/Resources/Images/servo.png\" alt=\"\" /></td>
</tr>
<tr>
<td><strong>&nbsp;</strong></td>
<td style=\"text-align: center;\">&nbsp;<span class=\"Apple-style-span\"><strong>Board Handlers</strong></span></td>
<td>&nbsp;</td>
</tr>
<tr>
<td>Arduino</td>
<td>Connects to Arduino boards like Arduino Uno, Arduino Mega 2560</td>
<td>&nbsp;<img style=\"display: block; margin-left: auto; margin-right: auto;\" src=\"Modelica://OpenModelicaArduino/Resources/Images/arduino.png\" alt=\"\" /></td>
</tr>
<tr>
<td>Arduino Leonardo</td>
<td>Connects to Arduino Leonardo boards and boards using native USB</td>
<td>&nbsp;<img style=\"display: block; margin-left: auto; margin-right: auto;\" src=\"Modelica://OpenModelicaArduino/Resources/Images/arduinoLeonardo.png\" alt=\"\" /></td>
</tr>
<tr>
<td>StandardFirmata</td>
<td>Connects to Arduino-compatible boards</td>
<td>&nbsp;<img style=\"display: block; margin-left: auto; margin-right: auto;\" src=\"Modelica://OpenModelicaArduino/Resources/Images/standard.png\" alt=\"\" /></td>
</tr>
<tr>
<td>CustomFirmata</td>
<td>Connects to any board supporting Firmata</td>
<td>&nbsp;<img style=\"display: block; margin-left: auto; margin-right: auto;\" src=\"Modelica://OpenModelicaArduino/Resources/Images/custom.png\" alt=\"\" /></td>
</tr>
</tbody>
</table>
<h4>How Does It Work?</h4>
<p>ModelPlug connects with the boards using USB serial communication. In order to configure, read, and write to the board, ModelPlug uses the Firmata protocol v2.3 (<a href=\"http://www.firmata.org/\">http://www.firmata.org</a>). This protocol allows you to connect not only to Arduinos, but also to many boards compatible with Arduino. Examples of other boards supporting the Firmata protocol are:</p>
<ul>
<li>Teensy Development Board: Using AVR or ARM processors</li>
<li>chipKIT: Using PIC32 processors</li>
</ul>
<p>ModelPlug wraps the functionality of Firmata by providing easy-to-use Modelica models that you can connect in your simulations.</p>
<h4>New in Version 1.2</h4>
<ul>
<li>Allows defining initial values in AnalogInput and DigitalInput Pins</li>
<li>Improved support for Windows 10</li>
</ul>
<h4>Limitations</h4>
<p>ModelPlug requires a board with Firmata Version 2.3 or higher.</p>
<p>Currently ModelPlug does not support sensors that communicate through I2C or SPI with the board.</p>
<p>The minimum synchronization interval is 1 ms; therefore, ModelPlug cannot have a sampling interval smaller than that. The standard Firmata allows a minimum&nbsp;sampling interval of 10 ms.<br /> <br />ModelPlug uses a serial protocol; therefore, the transfer speed is constrained by the serial port speed. This can be problematic when reading or writing too many inputs/outputs with a small sampling interval.</p>
<h4>Links</h4>
<p>Ask questions about SystemModeler and ModelPlug</p>
<p><a href=\"http://community.wolfram.com/\">http://community.wolfram.com</a></p>
<p>Find out how to connect sensors and actuators to the Arduino</p>
<p><a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a></p>
<p>Get details about the Firmata protocol</p>
<p><a href=\"http://www.firmata.org/\">http://www.firmata.org</a></p>
<p>The breadboard diagrams were created using Fritzing</p>
<p><a href=\"http://fritzing.org/\">http://fritzing.org</a></p>
<p>&nbsp;</p>
<p>&nbsp;</p>
<p>&nbsp;</p>", revisions = ""), version = "1.2", Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics = {Rectangle(visible = true, fillColor = {190, 53, 19}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}, radius = 25), Polygon(visible = true, origin = {-17.857, -4.643}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{42.857, 59.643}, {42.857, 64.643}, {37.857, 69.643}, {32.857, 69.643}, {-17.143, 69.643}, {-42.143, 54.643}, {-57.143, 34.643}, {-65.22199999999999, 4.643}, {-57.143, -25.357}, {-42.143, -45.357}, {-17.143, -60.357}, {32.857, -60.357}, {37.857, -60.357}, {42.857, -55.357}, {42.857, -50.357}}, smooth = Smooth.Bezier), Polygon(visible = true, origin = {-17.857, -4.643}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{42.857, 59.643}, {42.857, 64.643}, {37.857, 69.643}, {30.028, 54.643}, {-12.143, 59.643}, {-37.143, 44.643}, {-50.141, 26.339}, {-55.168, 4.643}, {-52.143, -20.357}, {-42.143, -42.453}, {-17.143, -60.357}, {32.857, -60.357}, {37.857, -60.357}, {42.857, -55.357}, {42.857, -50.357}}, smooth = Smooth.Bezier), Rectangle(visible = true, origin = {50, 27.5}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-25, -12.5}, {25, 12.5}}), Rectangle(visible = true, origin = {50, -27.5}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-25, -12.5}, {25, 12.5}}), Polygon(visible = true, origin = {-23.077, -0.385}, fillColor = {191, 191, 191}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{38.077, 50.385}, {38.077, 55.385}, {33.077, 55.385}, {-6.923, 55.385}, {-26.923, 45.385}, {-41.923, 30.385}, {-50.213, 0.385}, {-41.923, -29.615}, {-26.923, -44.615}, {-6.923, -54.615}, {33.077, -54.615}, {38.077, -54.615}, {38.077, -49.615}}, smooth = Smooth.Bezier), Polygon(visible = true, origin = {-17.857, -4.643}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, points = {{42.857, 59.643}, {42.857, 64.643}, {37.857, 69.643}, {32.857, 69.643}, {-17.143, 69.643}, {-42.143, 54.643}, {-57.143, 34.643}, {-65.22199999999999, 4.643}, {-57.143, -25.357}, {-42.143, -45.357}, {-17.143, -60.357}, {32.857, -60.357}, {37.857, -60.357}, {42.857, -55.357}, {42.857, -50.357}}, smooth = Smooth.Bezier)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})),
  uses(Modelica_DeviceDrivers(version = "1.5.0")));
end OpenModelicaArduino;
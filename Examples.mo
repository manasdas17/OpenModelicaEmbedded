  package Examples "A collection of examples to help you get started"
    extends Modelica.Icons.ExamplesPackage;

    model BlinkLed "Basic example of blinking an LED"
      extends Modelica.Icons.Example;
      replaceable OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0", ShowPinCapabilities = true, UseDTR = false) annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput(Pin = 30) annotation(
        Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {35, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse annotation(
        Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(digitalOutput.pinConnector, arduino.boardConnector) annotation(
        Line(points = {{10, -10}, {30, -10}}, color = {72, 73, 79}));
      connect(booleanPulse.y, digitalOutput.u) annotation(
        Line(points = {{-19, -10}, {-10, -10}}, color = {255, 0, 255}));
      annotation(
        experiment(StopTime = 10, Interval = 0.001),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware Components&nbsp;Used&nbsp;</h4>
<ul>
<li>1 Arduino board</li>
<li>1 LED (optional)</li>
<li>1 resistor 680 ohms (optional)</li>
</ul>
<h4>Description</h4>
<p>This example uses the DigitalOutput component to control the LED attached to the Arduino board on pin 13. It uses a BooleanPulse from the Modelica library to produce an On/Off signal that is fed into the DigitalOutput component. This will make the LED attached to the pin blink.</p>
<p>You can go ahead and add more LEDs to the board as shown in the following figure. This will require you to add one more DigitalOutput component to control the LED on pin 9.</p>
<p><img src=\"modelica://OpenModelicaArduino/Resources/Images/Blink.png\" alt=\"\"/></p></html>", revisions = ""),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end BlinkLed;

    model DimmingLed "Changing the intensity of an LED"
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0", ShowPinCapabilities = true) annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Pins.AnalogOutput analogOutput(MaxValue = 512, MinValue = -512, Pin = 30) annotation(
        Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {35, 25}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 512, MinValue = -512, Pin = 23) annotation(
        Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(analogInput1.pinConnector, arduino.boardConnector) annotation(
        Line(points = {{-40, -10}, {-45, -10}, {-45, -40}, {30, -40}, {30, -10}, {30, -10}}));
      connect(analogInput1.y, analogOutput.u) annotation(
        Line(points = {{-20, -10}, {-10, -10}, {-10, -10}, {-10, -10}}, color = {0, 0, 127}));
      connect(analogOutput.pinConnector, arduino.boardConnector) annotation(
        Line(visible = true, origin = {20, -10}, points = {{-10, -0}, {10, 0}}));
      annotation(
        experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <ul>
    <li>1 Arduino board</li>
    <li>1 LED</li>
    <li>1 resistor 680 ohms</li>
    </ul>
    <h4><br />Description</h4>
    <p>This example uses the AnalogOutput component to change the light intensity of an LED. AnalogOutput uses the Arduino function 'analogWrite', which produces a PWM (Pulse-Width Modulated) signal. This type of signal can be used to directly&nbsp;control the LED intensity. The following figure shows the connections.</p>
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/Dimming.png\" alt=\"\" /></p>
    <p>You can check the Arduino Playground to know more about PWM outputs.&nbsp;</p></html>", revisions = ""),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end DimmingLed;

    model ReadSensor "Reading analog signals"
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {0, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput(Pin = 10) annotation(
        Placement(visible = true, transformation(origin = {-30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(Pin = 16) annotation(
        Placement(visible = true, transformation(origin = {30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Greater greater1 annotation(
        Placement(visible = true, transformation(origin = {-10, -15}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Reference(k = 0.5) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(greater1.y, digitalOutput.u) annotation(
        Line(visible = true, origin = {-38.744, -2.5}, points = {{17.744, -12.5}, {-6.256, -12.5}, {-6.256, 17.5}, {-1.256, 17.5}}, color = {255, 0, 255}));
      connect(analogInput1.y, greater1.u1) annotation(
        Line(visible = true, origin = {33.994, -5}, points = {{6.006, 20}, {11.006, 20}, {11.006, 0}, {-23.994, 0}, {-23.994, -10}, {-31.994, -10}}, color = {0, 0, 127}));
      connect(analogInput1.pinConnector, arduino.boardConnector) annotation(
        Line(visible = true, origin = {10, 15}, points = {{10, 0}, {-10, 0}}, color = {72, 73, 79}));
      connect(digitalOutput.pinConnector, arduino.boardConnector) annotation(
        Line(visible = true, origin = {-10, 15}, points = {{-10, 0}, {10, 0}}, color = {72, 73, 79}));
      connect(Reference.y, greater1.u2) annotation(
        Line(visible = true, origin = {12.75, -29}, points = {{6.25, -1}, {-2.75, -1}, {-2.75, 6}, {-10.75, 6}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0.0, StopTime = 10.0, Interval = 0.001, __Wolfram_Algorithm = "dassl", Tolerance = 1e-006, __Wolfram_SynchronizeWithRealTime = true),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <ul>
    <li>1 Arduino board</li>
    <li>1 LED</li>
    <li>1 resistor 680 ohms</li>
    <li>1 potentiometer 100 K</li>
    </ul>
    <h4>Description</h4>
    <p>This example shows how to read an analog voltage using the AnalogInput component. The analog signal is then compared to a reference, and if the signal is above the reference, it will turn on an LED. You can see in the following figure how to build this example.</p>
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/ReadAnalog.png\" alt=\"\" /></p>
    <p>You can see that pin A0 for the Arduino corresponds to pin number 14 for the Firmata. For other boards, the pin numbering may vary.</p>
    <p>Run the simulation and move the potentiometer. You should see that when the position of the shaft is near the middle, the LED changes state.</p>
    <p>The AnalogInput component returns a signal between 0 and 1. This value represents the voltage between 0 and the reference voltage. If you prefer to get the signal directly in volts, you need to change the 'MaxValue' property to the reference voltage, but generally it is either 5 V or 3.3 V.</p></html>", revisions = ""),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end ReadSensor;

    model MoveServo "Using servos"
      extends Modelica.Icons.Example;
      Boards.Arduino arduino(Port = "COM1") annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Pins.Servo servo(Pin = 10) annotation(
        Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.ExpSine expSine(offset = 0.5, amplitude = 0.5, freqHz = 0.4, damping = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-25, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(expSine.y, servo.u) annotation(
        Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, -0}, {4.5, 0}}, color = {0, 0, 127}));
      connect(servo.pinConnector, arduino.boardConnector) annotation(
        Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, -0}}, color = {72, 73, 79}));
      annotation(
        experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <ul>
    <li>1 Arduino</li>
    <li>1 5 V servo</li>
    <li>1 external 5 V power source</li>
    </ul>
    <h4>Description</h4>
    <p>This example shows how to control a servo by using the Servo component. You can find the diagram in&nbsp;the following figure.</p>
    <p>For this example, it is recommended to use an external power source to provide voltage for the servo. This is because the power from the Arduino may not be enough to supply the servo. If you are not sure how to connect your servo, take a look at the reference in the Arduino Playground (<a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a>).</p>
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/ServoExample.png\" alt=\"\" /></p>
    <p>Servos are controlled with a signal in the range of 0 to 1, where 0 corresponds to 0 degrees of rotation and 1 to 180 degrees. This example makes the servo bounce from 0 to 180 degrees until it gets stable around 90 degrees.</p></html>", revisions = ""),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end MoveServo;

    model SimpleONOFF "A simple On/Off controller"
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {0, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 3.3 * 100, Pin = 23, adcResolution = 12) annotation(
        Placement(visible = true, transformation(origin = {30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Reference(k = 40) annotation(
        Placement(visible = true, transformation(origin = {35, -25}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
      Modelica.Blocks.Math.Add add1(k2 = +1, k1 = -1) annotation(
        Placement(visible = true, transformation(origin = {-0, -25}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Pins.DigitalOutput digitalOutput(Pin = 30) annotation(
        Placement(visible = true, transformation(origin = {-30, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Hysteresis hysteresis(uLow = -1, uHigh = 1) annotation(
        Placement(visible = true, transformation(origin = {-30, -25}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {42.5, 42.5}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0)));
    equation
      connect(hysteresis.u, add1.y) annotation(
        Line(visible = true, origin = {-14.5, -25}, points = {{-3.5, -0}, {3.5, 0}}, color = {0, 0, 127}));
      connect(digitalOutput.u, hysteresis.y) annotation(
        Line(visible = true, origin = {-43.25, 5}, points = {{3.25, 10}, {-1.75, 10}, {-1.75, -30}, {2.25, -30}}, color = {255, 0, 255}));
      connect(digitalOutput.pinConnector, arduino.boardConnector) annotation(
        Line(visible = true, origin = {-10, 15}, points = {{-10, 0}, {10, 0}}));
      connect(add1.u2, Reference.y) annotation(
        Line(visible = true, origin = {18.5, -23}, points = {{-6.5, -8}, {1.5, -8}, {1.5, -2}, {5.5, -2}}, color = {0, 0, 127}));
      connect(analogInput1.y, add1.u1) annotation(
        Line(visible = true, origin = {33.506, 12}, points = {{6.494, 3}, {11.494, 3}, {11.494, -17}, {-13.506, -17}, {-13.506, -31}, {-21.506, -31}}, color = {0, 0, 127}));
      connect(arduino.boardConnector, analogInput1.pinConnector) annotation(
        Line(visible = true, origin = {10, 15}, points = {{-10, 0}, {10, 0}}));
      annotation(
        experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware&nbsp;Components Used</h4>
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
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/SimpleONOFF.png\" alt=\"\" /></p>
    <p>The target temperature is set by a constant component. The measured temperature is subtracted from the reference in order to obtain the error. The error signal is fed into the hysteresis component, which will send a Boolean signal to control the relay. If you want cooling instead of heating, you need to invert the logic of this signal.</p></html>"),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end SimpleONOFF;

    model UsingArduinoLeonardo "Basic example of blinking an LED"
      extends OpenModelicaArduino.Examples.BlinkLed(redeclare ModelPlug.Boards.ArduinoLeonardo arduino);
      annotation(
        experiment(StopTime = 10, Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true),
        Documentation(info = "<html><h4>Hardware Components&nbsp;Used&nbsp;</h4>
   <ul>
   <li>1 Arduino Leonardo board</li>
   <li>1 LED (optional)</li>
   <li>1 resistor 680 ohms (optional)</li>
   </ul>
   <h4>Description</h4>
   <p>This example shows how the 
   <a href=\"modelica://OpenModelicaArduino.Examples.BlinkLed\">Blink Led</a> can be used with an Arduino Leonardo.</p></html>", revisions = ""));
    end UsingArduinoLeonardo;

    model UsingCustomBoard "Using a Firmata-compatible board"
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 1) annotation(
        Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Pins.DigitalOutput digitalOutput(Pin = 13) annotation(
        Placement(visible = true, transformation(origin = {-0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Boards.CustomFirmata customFirmata(Port = "COM1", BaudRate = 115200, UpdatePeriod = 0.001) annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(digitalOutput.pinConnector, customFirmata.boardConnector) annotation(
        Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, -0}}));
      connect(booleanPulse.y, digitalOutput.u) annotation(
        Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, 0}, {4.5, 0}}, color = {255, 0, 255}));
      annotation(
        experiment(Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware&nbsp;Components&nbsp;Used</h4>
    <p>- 1 Teensy 3.1 board</p>
    <h4>Description</h4>
    <p>This example shows how to use a board with a custom version of Firmata. The Teensy board works perfectly with the StandardFirmata, but in this example it is modified in order get a faster data transfer speed.</p>
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/CustomExample.png\" alt=\"\" /></p>
    <p><br />The main difference of the CustomFirmata component is that it allows you to set the baud rate to the sampling interval that you want to use. In this case the oce of the StandardFirmata is modified to use a baud rate of 115200. You can perform this modification in the source code. Search for the line:</p>
    <p><span style=\"font-family: 'courier new', courier;\">Firmata.Begin(57600);</span></p>
    <p>and changing it to</p>
    <p><span style=\"font-family: 'courier new', courier;\">Firmata.Begin(115200);</span></p>
    <p>This initializes the serial port at the given speed. The next change that you can make is reducing the minimum sampling interval. You can find this tweak by searching in the code the text:</p>
    <p><span style=\"font-family: 'courier new', courier;\">#define MINIMUM_SAMPLING_INTERVAL 10</span></p>
    <p>and changing it to</p>
    <p><span style=\"font-family: 'courier new', courier;\">#define MINIMUM_SAMPLING_INTERVAL 1</span></p>
    <p>This changes the sampling interval limit from 10 ms to 1ms. You have to consider that this change is possible because the Teensy board can run up to 96 MHz. If you change the sampling interval, it is necessary to change the simulation setting in SimulationCenter to 1 ms, as shown in the following figure.</p>
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/Interval.png\" alt=\"\" /></p></html>", revisions = ""),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end UsingCustomBoard;

    model UsingStandardFirmata "Using a standard Firmata board"
      extends Modelica.Icons.Example;
      Boards.StandardFirmata standardFirmata(Port = "COM1", ShowPinCapabilities = true) annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput(Pin = 13) annotation(
        Placement(visible = true, transformation(origin = {-0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 1) annotation(
        Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-35, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(digitalOutput.pinConnector, standardFirmata.boardConnector) annotation(
        Line(visible = true, origin = {20, -10}, points = {{-10, 0}, {10, 0}}, color = {72, 73, 79}));
      connect(booleanPulse.y, digitalOutput.u) annotation(
        Line(visible = true, origin = {-14.5, -10}, points = {{-4.5, 0}, {4.5, 0}}, color = {255, 0, 255}));
      annotation(
        experiment(StopTime = 10, Interval = 0.001, __Wolfram_SynchronizeWithRealTime = true),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10})),
        preferredView = "diagram",
        Documentation(info = "<html><h4>Hardware Components&nbsp;Used</h4>
    <ul>
    <li>Any board with standard Firmata (this example uses the Teensy board)</li>
    <li>1 LED (optional)</li>
    <li>1 resistor 680 ohms (optional)</li>
    </ul>
    <h4>Description</h4>
    <p>This example uses the DigitalOutput component to control the LED attached to a standard Firmata board on pin 13. It uses a BooleanPulse from the Modelica library to produce an On/Off signal that is fed into the DigitalOutput component. This will make the LED attached to the pin blink.</p>
    <p>You can go ahead and add more LEDs to the board as shown in the following figure. This will require you to add one more DigitalOutput component to control the LED on pin 9.</p>
    <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/StandardFirmata.png\" alt=\"\" /></p></html>", revisions = ""),
        Diagram(coordinateSystem(extent = {{-50, -50}, {50, 50}}, initialScale = 0.1, grid = {5, 5})));
    end UsingStandardFirmata;

    model DCMotorWithPWM
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {13, 29}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {54, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput1(MaxValue = 255, Pin = 30) annotation(
        Placement(visible = true, transformation(origin = {-57, 29}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 8) annotation(
        Placement(visible = true, transformation(origin = {-45, -23}, extent = {{-13, -13}, {13, 13}}, rotation = 90)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput2(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {50, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant1(k = true) annotation(
        Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant2(k = false) annotation(
        Placement(visible = true, transformation(origin = {14, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1024, Pin = 23) annotation(
        Placement(visible = true, transformation(origin = {-56, 74}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
    equation
      connect(analogInput1.y, analogOutput1.u) annotation(
        Line(points = {{-42, 74}, {-20, 74}, {-20, 52}, {-90, 52}, {-90, 28}, {-72, 28}, {-72, 30}}, color = {0, 0, 127}));
      connect(analogInput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-70, 74}, {-88, 74}, {-88, 92}, {12, 92}, {12, 30}, {14, 30}}));
      connect(digitalOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-45, -10}, {-44.75, -10}, {-44.75, -2}, {14, -2}, {14, 30}}));
      connect(booleanConstant1.y, digitalOutput1.u) annotation(
        Line(points = {{-58, -70}, {-46, -70}, {-46, -36}, {-45, -36}}, color = {255, 0, 255}));
      connect(booleanConstant2.y, digitalOutput2.u) annotation(
        Line(points = {{26, -74}, {50, -74}, {50, -40}, {49, -40}, {49, -38}, {50, -38}}, color = {255, 0, 255}));
      connect(digitalOutput2.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{50, -18}, {50, 30}, {14, 30}}));
      connect(analogOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-42, 30}, {13, 30}, {13, 29}}));
      annotation(
        uses(OpenModelicaArduino(version = "1.2"), Modelica(version = "3.2.2"), Modelica_DeviceDrivers(version = "1.5.0")));
    end DCMotorWithPWM;

    model DCMotorWithPID
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1(priority = "High", setPriority = true) annotation(
        Placement(visible = true, transformation(origin = {92, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1, a(start = 0), phi(start = 0), w(start = 0)) annotation(
        Placement(visible = true, transformation(origin = {-22, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia2(J = 1, a(start = 0), phi(start = 0), w(start = 0)) annotation(
        Placement(visible = true, transformation(origin = {56, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.SpringDamper springDamper1(c = 20, d = 10, phi_rel0 = 0) annotation(
        Placement(visible = true, transformation(origin = {16, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(
        Placement(visible = true, transformation(origin = {-64, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(tau_constant = 10) annotation(
        Placement(visible = true, transformation(origin = {98, -86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor1 annotation(
        Placement(visible = true, transformation(origin = {88, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      OpenModelicaArduino.Boards.CustomFirmata customFirmata1(BaudRate = 57600, Port = "/dev/ttyACM0", ShowPinCapabilities = true, UpdatePeriod = 0.02, UseDTR = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput1(MaxValue = 512, MinValue = -512, Pin = 5) annotation(
        Placement(visible = true, transformation(origin = {46, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(InitValue = 0, MaxValue = 512, MinValue = -512, Pin = 15) annotation(
        Placement(visible = true, transformation(origin = {-64, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput2(MaxValue = 512, MinValue = -512, Pin = 6) annotation(
        Placement(visible = true, transformation(origin = {-54, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Sine sine1(amplitude = 512, freqHz = 0.5) annotation(
        Placement(visible = true, transformation(origin = {-122, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sine1.y, analogOutput2.u) annotation(
        Line(points = {{-110, 58}, {-64, 58}, {-64, 58}, {-64, 58}}, color = {0, 0, 127}));
      connect(analogOutput2.pinConnector, customFirmata1.boardConnector) annotation(
        Line(points = {{-44, 58}, {-10, 58}, {-10, 4}}));
      connect(analogInput1.y, torque1.tau) annotation(
        Line(points = {{-74, 2}, {-84, 2}, {-84, 2}, {-94, 2}, {-94, -86}, {-76, -86}}, color = {0, 0, 127}));
      connect(analogInput1.pinConnector, customFirmata1.boardConnector) annotation(
        Line(points = {{-54, 2}, {-10, 2}, {-10, 4}}));
      connect(speedSensor1.w, analogOutput1.u) annotation(
        Line(points = {{88, -25}, {89, -25}, {89, -23}, {88, -23}, {88, 3}, {56, 3}, {56, 1}}, color = {0, 0, 127}));
      connect(analogOutput1.pinConnector, customFirmata1.boardConnector) annotation(
        Line(points = {{36, 2}, {35.5, 2}, {35.5, 2}, {35, 2}, {35, 4}, {-10, 4}}));
      connect(speedSensor1.flange, constantTorque1.flange) annotation(
        Line(points = {{88, -46}, {88, -86}}));
      connect(inertia2.flange_b, constantTorque1.flange) annotation(
        Line(points = {{66, -86}, {88, -86}, {88, -86}, {88, -86}, {88, -86}, {88, -86}}));
      connect(torque1.flange, inertia1.flange_a) annotation(
        Line(points = {{-54, -86}, {-37, -86}, {-37, -86}, {-32, -86}, {-32, -86}, {-31, -86}, {-31, -86}, {-32, -86}}));
      connect(springDamper1.flange_b, inertia2.flange_a) annotation(
        Line(points = {{26, -86}, {36, -86}, {36, -86}, {46, -86}, {46, -86}, {46, -86}}));
      connect(inertia1.flange_b, springDamper1.flange_a) annotation(
        Line(points = {{-12, -86}, {-3, -86}, {-3, -86}, {6, -86}, {6, -86}, {6, -86}}));
      annotation(
        uses(OpenModelicaArduino(version = "1.2"), Modelica(version = "3.2.2"), Modelica_DeviceDrivers(version = "1.5.0")));
    end DCMotorWithPID;
    annotation(
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end Examples;
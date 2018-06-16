within OpenModelicaArduino;

package ArduinoExamples
  extends Modelica.Icons.ExamplesPackage;

  package Led
    extends Modelica.Icons.ExamplesPackage;

    model ex1_led_blue
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant1(k = true) annotation(
        Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0")  annotation(
        Placement(visible = true, transformation(origin = {61, -21}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
    equation
      connect(digitalOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{20, -20}, {60, -20}, {60, -20}, {62, -20}}));
      connect(booleanConstant1.y, digitalOutput1.u) annotation(
        Line(points = {{-38, -20}, {-20, -20}, {-20, -20}, {-20, -20}}, color = {255, 0, 255}));
    end ex1_led_blue;

    model ex2_led_blue_delay
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {60, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1(enableRealTimeScaling = false) annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 20, startTime = 5, width = 10) annotation(
        Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(booleanPulse1.y, digitalOutput1.u) annotation(
        Line(points = {{-38, -20}, {-20, -20}, {-20, -20}, {-20, -20}}, color = {255, 0, 255}));
      connect(digitalOutput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{20, -20}, {60, -20}, {60, -20}, {60, -20}}));
    end ex2_led_blue_delay;

    model ex3_led_blue_red
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput2(Pin = 11) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 20, startTime = 5, width = 25) annotation(
        Placement(visible = true, transformation(origin = {-60, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse2(period = 20, startTime = 5, width = 40) annotation(
        Placement(visible = true, transformation(origin = {-60, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpenModelicaArduino.Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0")  annotation(
        Placement(visible = true, transformation(origin = {60, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(digitalOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{20, 0}, {60, 0}, {60, -20}, {60, -20}}));
      connect(digitalOutput2.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{20, -60}, {60, -60}, {60, -20}, {60, -20}}));
      connect(booleanPulse2.y, digitalOutput2.u) annotation(
        Line(points = {{-38, -60}, {-20, -60}, {-20, -60}, {-20, -60}}, color = {255, 0, 255}));
      connect(booleanPulse1.y, digitalOutput1.u) annotation(
        Line(points = {{-38, 0}, {-20, 0}, {-20, 0}, {-20, 0}}, color = {255, 0, 255}));
    end ex3_led_blue_red;

    model ex4_led_blink
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {42, -6}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 10) annotation(
        Placement(visible = true, transformation(origin = {-9, -7}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 2, startTime = 4, width = 50) annotation(
        Placement(visible = true, transformation(origin = {-68, -12}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-19, 51}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
    equation
      connect(booleanPulse1.y, digitalOutput1.u) annotation(
        Line(points = {{-53, -12}, {-24, -12}, {-24, -7}, {-22, -7}}, color = {255, 0, 255}));
      connect(digitalOutput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{4, -7}, {22, -7}, {22, -6}, {42, -6}}));
      annotation(
        uses(OpenModelicaArduino(version = "1.2"), Modelica(version = "3.2.2"), Modelica_DeviceDrivers(version = "1.5.0")));
    end ex4_led_blink;
  end Led;

  package Push_button
    extends Modelica.Icons.ExamplesPackage;

    model ex1_push_button_status
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {-60, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalInput digitalInput1(Pin = 12) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.BooleanValue booleanValue1 annotation(
        Placement(visible = true, transformation(origin = {80, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(digitalInput1.y, booleanValue1.activePort) annotation(
        Line(points = {{20, 0}, {54, 0}, {54, 0}, {58, 0}}, color = {255, 0, 255}));
      connect(arduino1.boardConnector, digitalInput1.pinConnector) annotation(
        Line(points = {{-60, 0}, {-20, 0}}));
      Modelica.Utilities.Streams.print(String(booleanValue1.activePort));
    end ex1_push_button_status;

    model ex2_led_push_button
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalInput digitalInput1(Pin = 12) annotation(
        Placement(visible = true, transformation(origin = {62, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpenModelicaArduino.Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0")  annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(customBoard1.boardConnector, digitalInput1.pinConnector) annotation(
        Line(points = {{0, 0}, {44, 0}, {44, 0}, {42, 0}}));
      connect(digitalOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-40, 0}, {0, 0}, {0, 0}, {0, 0}}));
      connect(digitalInput1.y, digitalOutput1.u) annotation(
        Line(points = {{82, 0}, {94, 0}, {94, -40}, {-92, -40}, {-92, 0}, {-80, 0}, {-80, 0}}, color = {255, 0, 255}));
    end ex2_led_push_button;
  end Push_button;

  package Ldr
    extends Modelica.Icons.ExamplesPackage;

    model ex1_ldr_read
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1024, MinValue = 0, Pin = 23, adcResolution = 12) annotation(
        Placement(visible = true, transformation(origin = {-4.44089e-16, 4.44089e-16}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interaction.Show.RealValue realValue1(use_numberPort = true)  annotation(
        Placement(visible = true, transformation(origin = {60, -8.88178e-16}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(analogInput1.y, realValue1.numberPort) annotation(
        Line(points = {{20, 0}, {37, 0}}, color = {0, 0, 127}));
      connect(arduino1.boardConnector, analogInput1.pinConnector) annotation(
        Line(points = {{-60, 0}, {-20, 0}, {-20, 0}, {-20, 0}}));
      Modelica.Utilities.Streams.print(String(realValue1.numberPort));
      annotation(
        uses(Modelica(version = "3.2.2")));
    end ex1_ldr_read;




    model ex2_ldr_led
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1023, MinValue = 0, Pin = 19) annotation(
        Placement(visible = true, transformation(origin = {60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-70, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 300) annotation(
        Placement(visible = true, transformation(origin = {60, -70}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
      Modelica.Blocks.Logical.Less less1 annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -40}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
  OpenModelicaArduino.Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0")  annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(customBoard1.boardConnector, analogInput1.pinConnector) annotation(
        Line(points = {{0, 20}, {40, 20}, {40, 20}, {40, 20}}));
      connect(digitalOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-40, 20}, {0, 20}, {0, 20}, {0, 20}}));
      connect(const.y, less1.u2) annotation(
        Line(points = {{38, -70}, {32, -70}, {32, -56}, {24, -56}}, color = {0, 0, 127}));
      connect(less1.y, digitalOutput1.u) annotation(
        Line(points = {{-22, -40}, {-92, -40}, {-92, 20}, {-80, 20}}, color = {255, 0, 255}));
      connect(analogInput1.y, less1.u1) annotation(
        Line(points = {{80, 20}, {90, 20}, {90, -40}, {24, -40}}, color = {0, 0, 127}));
    end ex2_ldr_led;

  end Ldr;

  package DC_motor
    extends Modelica.Icons.ExamplesPackage;

    model ex1_dcmotor_clock
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {60, 4.21885e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput1(MaxValue = 255, MinValue = 0, Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput2(MaxValue = 255, MinValue = 0, Pin = 10) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 0) annotation(
        Placement(visible = true, transformation(origin = {-60, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Pulse pulse1(amplitude = 255, period = 20, startTime = 5, width = 15) annotation(
        Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(analogOutput2.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{20, -40}, {60, -40}, {60, 0}, {60, 0}}));
      connect(analogOutput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{20, 20}, {60, 20}, {60, 0}, {60, 0}, {60, 0}}));
      connect(const.y, analogOutput2.u) annotation(
        Line(points = {{-38, -40}, {-22, -40}, {-22, -40}, {-20, -40}}, color = {0, 0, 127}));
      connect(pulse1.y, analogOutput1.u) annotation(
        Line(points = {{-38, 20}, {-22, 20}, {-22, 20}, {-20, 20}}, color = {0, 0, 127}));
    end ex1_dcmotor_clock;

    model ex2_dcmotor_both
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {60, 4.21885e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 8)  annotation(
        Placement(visible = true, transformation(origin = {1.33227e-15, 26}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  OpenModelicaArduino.Pins.DigitalOutput digitalOutput2(Pin = 9)  annotation(
        Placement(visible = true, transformation(origin = {4, -40}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanPulse booleanPulse1(period = 20)  annotation(
        Placement(visible = true, transformation(origin = {-65, 25}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanPulse booleanPulse2(period = 20, startTime = 10)  annotation(
        Placement(visible = true, transformation(origin = {-60, -40}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
    equation
      connect(booleanPulse1.y, digitalOutput1.u) annotation(
        Line(points = {{-50, 26}, {-14, 26}}, color = {255, 0, 255}));
      connect(digitalOutput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{14, 26}, {60, 26}, {60, 0}}));
    connect(booleanPulse2.y, digitalOutput2.u) annotation(
        Line(points = {{-44, -40}, {-10, -40}}, color = {255, 0, 255}));
    connect(digitalOutput2.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{18, -40}, {60, -40}, {60, 0}}));
    end ex2_dcmotor_both;

    model ex3_dcmotor_loop
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {60, 4.21885e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput1(MaxValue = 255, MinValue = 0, Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 18}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogOutput analogOutput2(MaxValue = 255, MinValue = 0, Pin = 10) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Pulse pulse1(amplitude = 255, period = 10, startTime = 5, width = 30) annotation(
        Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Pulse pulse2(amplitude = 255, period = 10, startTime = 10, width = 20) annotation(
        Placement(visible = true, transformation(origin = {-60, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(pulse1.y, analogOutput1.u) annotation(
        Line(points = {{-38, 20}, {-20, 20}, {-20, 18}}, color = {0, 0, 127}));
      connect(analogOutput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{20, 18}, {20, 39}, {60, 39}, {60, 0}}));
      connect(pulse2.y, analogOutput2.u) annotation(
        Line(points = {{-38, -40}, {-20, -40}, {-20, -40}, {-20, -40}}, color = {0, 0, 127}));
      connect(analogOutput2.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{20, -40}, {60, -40}, {60, 0}, {60, 0}}));
    end ex3_dcmotor_loop;
  end DC_motor;

  package Potentiometer
    extends Modelica.Icons.ExamplesPackage;

    model ex1_pot_threshold
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1023, MinValue = 0, Pin = 23, adcResolution = 12) annotation(
        Placement(visible = true, transformation(origin = {28, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput2(Pin = 10) annotation(
        Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold1(threshold = 0) annotation(
        Placement(visible = true, transformation(origin = {70, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold2(threshold = 320) annotation(
        Placement(visible = true, transformation(origin = {70, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold3(threshold = 900) annotation(
        Placement(visible = true, transformation(origin = {70, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Xor xor1 annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Logical.Xor xor2 annotation(
        Placement(visible = true, transformation(origin = {30, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput3(Pin = 11) annotation(
        Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpenModelicaArduino.Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0")  annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
    equation
      connect(analogInput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{18, 50}, {-10, 50}}));
      connect(digitalOutput1.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-40, 90}, {-10, 90}, {-10, 50}}));
      connect(digitalOutput2.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-40, 50}, {-10, 50}}));
      connect(digitalOutput3.pinConnector, customBoard1.boardConnector) annotation(
        Line(points = {{-40, 10}, {-10, 10}, {-10, 50}}));
      connect(xor1.y, digitalOutput3.u) annotation(
        Line(points = {{20, -10}, {-72, -10}, {-72, 10}, {-60, 10}, {-60, 10}}, color = {255, 0, 255}));
      connect(xor2.y, digitalOutput2.u) annotation(
        Line(points = {{20, -50}, {-80, -50}, {-80, 50}, {-60, 50}, {-60, 50}}, color = {255, 0, 255}));
      connect(greaterEqualThreshold3.y, digitalOutput1.u) annotation(
        Line(points = {{60, -70}, {-90, -70}, {-90, 90}, {-60, 90}, {-60, 90}}, color = {255, 0, 255}));
      connect(greaterEqualThreshold3.y, xor2.u2) annotation(
        Line(points = {{60, -70}, {52, -70}, {52, -58}, {42, -58}, {42, -58}, {42, -58}}, color = {255, 0, 255}));
      connect(greaterEqualThreshold2.y, xor2.u1) annotation(
        Line(points = {{60, -30}, {52, -30}, {52, -50}, {42, -50}, {42, -50}}, color = {255, 0, 255}));
      connect(greaterEqualThreshold2.y, xor1.u2) annotation(
        Line(points = {{60, -30}, {52, -30}, {52, -18}, {42, -18}, {42, -18}}, color = {255, 0, 255}));
      connect(greaterEqualThreshold1.y, xor1.u1) annotation(
        Line(points = {{60, 10}, {50, 10}, {50, -10}, {42, -10}, {42, -10}}, color = {255, 0, 255}));
      connect(analogInput1.y, greaterEqualThreshold1.u) annotation(
        Line(points = {{38, 50}, {94, 50}, {94, 10}, {82, 10}, {82, 10}}, color = {0, 0, 127}));
      connect(analogInput1.y, greaterEqualThreshold2.u) annotation(
        Line(points = {{38, 50}, {94, 50}, {94, -30}, {82, -30}, {82, -30}, {82, -30}}, color = {0, 0, 127}));
      connect(analogInput1.y, greaterEqualThreshold3.u) annotation(
        Line(points = {{38, 50}, {94, 50}, {94, -70}, {82, -70}, {82, -70}}, color = {0, 0, 127}));
      annotation(
        uses(OpenModelicaArduino(version = "1.2")));
    end ex1_pot_threshold;
  end Potentiometer;

  package Thermistor
    extends Modelica.Icons.ExamplesPackage;

    model ex1_therm_read
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1023, MinValue = 0, Pin = 18) annotation(
        Placement(visible = true, transformation(origin = {-4.44089e-16, 4.44089e-16}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpenModelicaArduino.Boards.customBoard customBoard1(BoardName = "Arduino UNO", Port = "/dev/ttyACM0")  annotation(
        Placement(visible = true, transformation(origin = {-59, -1}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
    equation
      connect(customBoard1.boardConnector, analogInput1.pinConnector) annotation(
        Line(points = {{-58, 0}, {-20, 0}, {-20, 0}, {-20, 0}}));
      Modelica.Utilities.Streams.print(String(analogInput1.y));
    end ex1_therm_read;

    model ex2_therm_buzzer
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.DigitalOutput digitalOutput1(Pin = 3) annotation(
        Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1023, MinValue = 0, Pin = 18) annotation(
        Placement(visible = true, transformation(origin = {60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-70, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 500) annotation(
        Placement(visible = true, transformation(origin = {60, -70}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
      Modelica.Blocks.Logical.Greater greater1 annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, -40}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
      Modelica.Blocks.Interaction.Show.RealValue realValue1 annotation(
        Placement(visible = true, transformation(origin = {40, 60}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
    equation
      connect(analogInput1.y, realValue1.numberPort) annotation(
        Line(points = {{80, 20}, {90, 20}, {90, 60}, {64, 60}, {64, 60}}, color = {0, 0, 127}));
      connect(greater1.y, digitalOutput1.u) annotation(
        Line(points = {{-22, -40}, {-94, -40}, {-94, 20}, {-80, 20}, {-80, 20}}, color = {255, 0, 255}));
      connect(analogInput1.y, greater1.u1) annotation(
        Line(points = {{80, 20}, {92, 20}, {92, -40}, {24, -40}, {24, -40}}, color = {0, 0, 127}));
      connect(greater1.u2, const.y) annotation(
        Line(points = {{24, -56}, {32, -56}, {32, -70}, {38, -70}, {38, -70}}, color = {0, 0, 127}));
      connect(arduino1.boardConnector, analogInput1.pinConnector) annotation(
        Line(points = {{0, 20}, {40, 20}}));
      connect(digitalOutput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{-40, 20}, {0, 20}}));
      Modelica.Utilities.Streams.print(String(realValue1.numberPort));
      annotation(
        uses(Modelica(version = "3.2.2")));
    end ex2_therm_buzzer;
  end Thermistor;

  package Servo_motor
    extends Modelica.Icons.ExamplesPackage;

    model ex1_servo_init
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {78, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.Servo servo1(InputUnit = OpenModelicaArduino.Internal.Types.ServoUnit.None, MaxPulse = 1000000, MinPulse = 500000, Pin = 23) annotation(
        Placement(visible = true, transformation(origin = {20, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 0.1667) annotation(
        Placement(visible = true, transformation(origin = {-40, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(const.y, servo1.u) annotation(
        Line(points = {{-18, 20}, {-4, 20}, {-4, 20}, {0, 20}}, color = {0, 0, 127}));
      connect(servo1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{40, 20}, {76, 20}, {76, 18}, {78, 18}, {78, 20}}));
      annotation(
        uses(Modelica_DeviceDrivers(version = "1.5.0"), OpenModelicaArduino(version = "1.2"), Modelica(version = "3.2.2")));
    end ex1_servo_init;


    model ex2_servo_reverse
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {78, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.Servo servo1(InputUnit = OpenModelicaArduino.Internal.Types.ServoUnit.None, MaxPulse = 1000000, MinPulse = 500000, Pin = 3) annotation(
        Placement(visible = true, transformation(origin = {20, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Pulse pulse2(amplitude = 0.5, offset = 0.5, period = 2, startTime = 6, width = 50) annotation(
        Placement(visible = true, transformation(origin = {-80, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Math.Product product1 annotation(
        Placement(visible = true, transformation(origin = {-40, 1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 0.5) annotation(
        Placement(visible = true, transformation(origin = {-80, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(const.y, product1.u1) annotation(
        Line(points = {{-58, 60}, {-36, 60}, {-36, 24}, {-80, 24}, {-80, 10}, {-64, 10}, {-64, 12}}, color = {0, 0, 127}));
      connect(pulse2.y, product1.u2) annotation(
        Line(points = {{-58, -60}, {-48, -60}, {-48, -24}, {-80, -24}, {-80, -12}, {-64, -12}, {-64, -12}}, color = {0, 0, 127}));
      connect(product1.y, servo1.u) annotation(
        Line(points = {{-18, 0}, {0, 0}, {0, 0}, {0, 0}}, color = {0, 0, 127}));
      connect(servo1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{40, 0}, {78, 0}}));
      annotation(
        uses(Modelica_DeviceDrivers(version = "1.5.0"), OpenModelicaArduino(version = "1.2"), Modelica(version = "3.2.2")));
    end ex2_servo_reverse;

    model ex3_servo_loop
      extends Modelica.Icons.Example;
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {78, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.Servo servo1(InputUnit = OpenModelicaArduino.Internal.Types.ServoUnit.None, MaxPulse = 1000000, MinPulse = 500000, Pin = 23) annotation(
        Placement(visible = true, transformation(origin = {20, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp1(duration = 10, height = 10, startTime = 5) annotation(
        Placement(visible = true, transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
        Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 0.11) annotation(
        Placement(visible = true, transformation(origin = {-70, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Product product1 annotation(
        Placement(visible = true, transformation(origin = {-28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.IntegerToReal integerToReal1 annotation(
        Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(integerToReal1.y, product1.u2) annotation(
        Line(points = {{2, -70}, {14, -70}, {14, -36}, {-56, -36}, {-56, -6}, {-40, -6}, {-40, -6}}, color = {0, 0, 127}));
      connect(realToInteger1.y, integerToReal1.u) annotation(
        Line(points = {{-38, -70}, {-22, -70}, {-22, -70}, {-22, -70}}, color = {255, 127, 0}));
      connect(ramp1.y, realToInteger1.u) annotation(
        Line(points = {{-78, -70}, {-62, -70}}, color = {0, 0, 127}));
      connect(const.y, product1.u1) annotation(
        Line(points = {{-58, 68}, {-52, 68}, {-52, 6}, {-40, 6}, {-40, 6}}, color = {0, 0, 127}));
      connect(product1.y, servo1.u) annotation(
        Line(points = {{-16, 0}, {0, 0}, {0, 0}, {0, 0}}, color = {0, 0, 127}));
      connect(servo1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{40, 0}, {78, 0}}));
      annotation(
        uses(Modelica_DeviceDrivers(version = "1.5.0"), OpenModelicaArduino(version = "1.2"), Modelica(version = "3.2.2")));
    end ex3_servo_loop;



    model ex4_servo_pot
      extends Modelica.Icons.Example;
      OpenModelicaArduino.Boards.Arduino arduino1(Port = "/dev/ttyACM0") annotation(
        Placement(visible = true, transformation(origin = {60, 3.10862e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.Servo servo1(InputUnit = OpenModelicaArduino.Internal.Types.ServoUnit.None, MaxPulse = 1000000, MinPulse = 500000, Pin = 9) annotation(
        Placement(visible = true, transformation(origin = {-3.55271e-15, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OpenModelicaArduino.Pins.AnalogInput analogInput1(MaxValue = 1, MinValue = 0, Pin = 16) annotation(
        Placement(visible = true, transformation(origin = {-60, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(analogInput1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{-80, 0}, {-92, 0}, {-92, -52}, {60, -52}, {60, 0}, {60, 0}}));
      connect(servo1.pinConnector, arduino1.boardConnector) annotation(
        Line(points = {{20, 0}, {62, 0}, {62, 0}, {60, 0}, {60, 0}}));
      connect(analogInput1.y, servo1.u) annotation(
        Line(points = {{-40, 0}, {-16, 0}, {-16, 0}, {-20, 0}}, color = {0, 0, 127}));
      annotation(
        uses(OpenModelicaArduino(version = "1.2"), Modelica_DeviceDrivers(version = "1.5.0")));
    end ex4_servo_pot;
  end Servo_motor;
end ArduinoExamples;
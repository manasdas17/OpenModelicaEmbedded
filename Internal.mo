  package Internal "Internal classes that should not be used directly by the user"
    extends Modelica.Icons.Package;

    package Icons "Icons used in ModelPlug"
      extends Modelica.Icons.IconsPackage;

      partial model Block "Icon for blocks"
        annotation(
          experiment(NumberOfIntervals = -1),
          preferredView = "icon",
          Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -80}, {100, 80}}, radius = 40), Rectangle(visible = true, origin = {5, -5}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-95, -75}, {95, 75}}, radius = 40), Rectangle(visible = true, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-90, -70}, {90, 70}}, radius = 30)}),
          Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
      end Block;
      annotation(
        Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end Icons;

    package ExternalFunctions "External functions used to communicate with the boards"
      extends Modelica.Icons.Package;

      function readAnalogPin
        input Integer pin;
        input Real min;
        input Real max;
        input Real init;
        input Integer board;
        input Integer adcResolution;
        output Real value;
      
        external "C" value = readAnalogPin(pin, min, max, init, board, adcResolution) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
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
        
          external "C" board = boardConstructor(port, showCapabilities, samplingMs, BaudRate, UseDTR) annotation(
            Include = "#include \"modelPlugFirmata.h\"",
            Library = "modelPlugFirmata",
            IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
            LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
        end constructor;

        function destructor "Function to destroy the object"
          input FirmataBoardObject board;
        
          external "C" boardDestructor(board) annotation(
            Include = "#include \"modelPlugFirmata.h\"",
            Library = "modelPlugFirmata",
            IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
            LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
        end destructor;
      end FirmataBoardObject;

      function getBoardId
        input FirmataBoardObject board;
        output Integer id;
      
        external "C" id = getBoardId(board) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end getBoardId;

      function readDigitalPin
        input Integer pin;
        input Boolean init;
        input Integer board;
        output Boolean value;
      
        external "C" value = readDigitalPin(pin, init, board) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end readDigitalPin;

      function writeAnalogPin
        input Integer pin;
        input Integer board;
        input Real value;
      
        external "C" writeAnalogPin(pin, board, value) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end writeAnalogPin;

      function writeDigitalPin
        input Integer pin;
        input Integer board;
        input Boolean value;
      
        external "C" writeDigitalPin(pin, board, value) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end writeDigitalPin;

      function updateBoard
        input Integer board;
      
        external "C" updateBoard(board) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end updateBoard;

      function writeServoPin
        input Integer pin;
        input Integer board;
        input Real value;
        input Integer MinPulse;
        input Integer MaxPulse;
      
        external "C" writeServoPin(pin, board, value, MinPulse, MaxPulse) annotation(
          Include = "#include \"modelPlugFirmata.h\"",
          Library = "modelPlugFirmata",
          IncludeDirectory = "modelica://OpenModelicaArduino/Resources/Include",
          LibraryDirectory = "modelica://OpenModelicaArduino/Resources/Library");
      end writeServoPin;
      annotation(
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {11.425, 9.596}, extent = {{-101.424, -59.596}, {78.57599999999999, 40.404}}, textString = "EF")}),
        Documentation(info = "", revisions = ""),
        Diagram(coordinateSystem(extent = {{-148.5, 105}, {148.5, -105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(visible = true, fillColor = {209, 209, 209}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {75, 75}}), Polygon(visible = true, fillColor = {236, 236, 236}, fillPattern = FillPattern.Solid, points = {{-100, 75}, {-75, 100}, {100, 100}, {75, 75}}), Polygon(visible = true, fillColor = {177, 177, 177}, fillPattern = FillPattern.Solid, points = {{75, -100}, {75, 75}, {100, 100}, {100, -75}}), Text(visible = true, extent = {{-95.95, -91.88}, {63.97, 71.52}}, textString = "C")}));
    end ExternalFunctions;

    package Types "Type and unit definitions"
      extends Modelica.Icons.TypesPackage;
      type ServoUnit = enumeration(None, Degrees, Radians);
      type SerialPort = String annotation(
        Dialog(__Wolfram_serialPortSelector = true));
      annotation(
        Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end Types;

    package Interfaces "Package with connectors and partial models"
      extends Modelica.Icons.InterfacesPackage;
      connector BoardConnector = output Integer annotation(
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, origin = {-0, 0.74}, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-10, -9.26}, {10, 9.26}})}),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, origin = {-0, 0.74}, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-10, -9.26}, {10, 9.26}})}));
      connector PinConnector = input Integer annotation(
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Ellipse(visible = true, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -70}, {70, 70}})}),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Ellipse(visible = true, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -70}, {70, 70}})}));
    end Interfaces;
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {1.383, -4.142}, rotation = 45, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-15, 93.333}, {-15, 68.333}, {0, 58.333}, {15, 68.333}, {15, 93.333}, {20, 93.333}, {25, 83.333}, {25, 58.333}, {10, 43.333}, {10, -41.667}, {25, -56.667}, {25, -76.667}, {10, -91.667}, {0, -91.667}, {0, -81.667}, {5, -81.667}, {15, -71.667}, {15, -61.667}, {5, -51.667}, {-5, -51.667}, {-15, -61.667}, {-15, -71.667}, {-5, -81.667}, {0, -81.667}, {0, -91.667}, {-10, -91.667}, {-25, -76.667}, {-25, -56.667}, {-10, -41.667}, {-10, 43.333}, {-25, 58.333}, {-25, 83.333}, {-20, 93.333}}), Polygon(visible = true, origin = {10.102, 5.218}, rotation = -45, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-15, 87.273}, {15, 87.273}, {20, 82.273}, {20, 27.273}, {10, 17.273}, {10, 7.273}, {20, 2.273}, {20, -2.727}, {5, -2.727}, {5, -77.727}, {10, -87.727}, {5, -112.727}, {-5, -112.727}, {-10, -87.727}, {-5, -77.727}, {-5, -2.727}, {-20, -2.727}, {-20, 2.273}, {-10, 7.273}, {-10, 17.273}, {-20, 27.273}, {-20, 82.273}})}));
  end Internal;
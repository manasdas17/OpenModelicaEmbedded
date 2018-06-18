  package GettingStarted "Procedure to run first model using OpenModelicaArduino"
    extends Modelica.Icons.Information;
    annotation(
      preferredView = "info",
      Documentation(info = "<html> <p> .</p>
      <h4>Preparing Your Board</h4>
      <p> .</p>
      <p>The first thing you need to do is upload the Firmware code to your board .</p>
      <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/FirmataLocation.png\" alt=\"\" /></p>
      <p><em>Figure 1. Location of the Firmware sketch.</em></p>
      <p>If you wish to use PID(Proportional-integral-derivative) then use another firmware provided with the package. To open an firmware through Arduino IDE click on &ldquo;Open&ldquo; and Browse to the location where you extracted the package. Insdie the package folder look for folder named &ldquo;Firmware&ldquo; which contains a file named &ldquo;pidmata3.ino&ldquo;. Select that file and click open to load firmware in Arduino IDE.<br>
      Important instructions for using above mentioned firmware &ldquo;pidmata3.ino&ldquo;:
      <ul>
      <li>To use PID uncomment the macro &ldquo;#define PID&ldquo; in firmware and follow further instructions in firmware.</li>
      <li>If you are not using PID then comment the above mentioned macro.</li>
      </ul>
      </p>
      <p>Once the the Firmware code is in the board, you&nbsp;need to write down the serial port that it is using. This is important because you&nbsp;need to give the port name to OpenModelicaArduino in order to communicate with the board. You can find the serial port in Tools-&gt;Serial Port or in the bottom-right corner of the Arduino software window (see Figure 2). In Windows the serial port name is something like &ldquo;COM5&rdquo;, while in OS X and Linux the name will be something like &ldquo;/dev/ttyACM0&rdquo;. Now you&nbsp;are ready to make your&nbsp;first model.</p>
      <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/SerialPortLocation.png\" alt=\"\" /></p>
      <p><em>Figure 2. Finding the serial port being used.</em>&nbsp;</p>
      <h4>Blinking LED</h4>
      <p>As a first exercise, you&nbsp;are going to reproduce with OpenModelicaArduino the blinking LED example. You can either open the prebuilt example (OpenModelicaArduino.Examples.BlinkLed) or build it by yourself. To build the model, locate the components:</p>
      <ul>
      <li>OpenModelicaArduino.Pins.DigitalOutput</li>
      <li>OpenModelicaArduino.Boards.Arduino</li>
      <li>Modelica.Blocks.Sources.BooleanPulse</li>
      </ul>
      <p>Connect the components as in Figure 3.&nbsp;One thing to notice is that the DigitalOutput model is connected to the Arduino component to specify that the pin belongs to that board. This connection is necessary because OpenModelicaArduino can use multiple boards with multiple pins at the same time.</p>
      <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/BlinkingLEDModel.png\" alt=\"\" /></p>
      <p><em>Figure 3. Diagram of the blinking LED.</em></p>
      <p>Next you&nbsp;need to specify the serial port that the board is using. This is done by selecting the Arduino component and showing its parameters. In the parameter view, you&nbsp;can find the Port parameter. Write the port name that you got in the section &ldquo;Preparing your board&rdquo;. Important: the name must be have quotation marks&nbsp;as shown in Figure 4.</p>
      <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/QuotedSerialPort.png\" alt=\"\" /></p>
      <p><em>Figure 4. Specifying the serial port name.&nbsp;</em></p>
      <p>Now you&nbsp;need to set the pin number that you&nbsp;are going to use in the DigitalOutput component. Usually, the Arduino boards have an LED attached to pin 13. Set that number in the Pin parameter as shown in Figure 5.</p>
      <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/PinNumber.png\" alt=\"\" /></p>
      <p><em>Figure 5. Specifying the pin to use.</em></p>
      <p>For the BooleanPulse component, set the 'period' parameter to 1. The model is ready to simulate. Press the simulate button and wait to see the results.</p>
      <p>The first time you run the model, it is probably simulated so fast that you do not have&nbsp;time to react. The reason is that OpenModelica simulates the model as fast as possible. In order to interact with your models using OpenModelicaArduino, it is necessary to synchronize the simulation time with real time. This is done in by using Synchronisation block in ModelicaDeviceDriver library. After simulating the model one time, you need to check in the checkbox&nbsp;&ldquo;Synchronize with real-time&rdquo; as shown in Figure 6.</p>
      //<p>&nbsp;<img src=\"modelica://OpenModelicaArduino/Resources/Images/SynchronizeSetting.png\" alt=\"\" /></p>
      //<p><em>Figure 6. Synchronizing your simulation with real time.</em>&nbsp;</p>
      //<p>If you are building many models with the OpenModelicaArduino library, you should set the &ldquo;Synchronize with real-time&rdquo; option as a default simulation setting in SimulationCenter under the menu Tools-&gt;Options in the section SimulationCenter-&gt;Default Experiment (see Figure 7).&nbsp;</p>
      //<p><img src=\"modelica://OpenModelicaArduino/Resources/Images/DefaultSynchronize.png\" alt=\"\" />&nbsp;</p>
      //<p><em>Figure 7. Setting \"Synchronize with real-time\" as default.</em></p>
      //<p>Now run the simulation again and you should see the LED blinking until the simulation reaches the stop time. If you want to keep the simulation running continuously, you need to change the stop time to &ldquo;infinite&rdquo; as shown in Figure 8.</p>
      //<p><img src=\"modelica://OpenModelicaArduino/Resources/Images/StopTime.png\" alt=\"\" /></p>
      //<p><em>Figure 8. &nbsp;Setting the stop time to infinite.</em></p>
      //<p>If you run the simulation again, you should see the LED blinking continuously.</p>
      <p>One thing that you may have noticed is that OpenModelicaArduino prints status messages in the simulation log (see Figure 9). The first thing it prints is a list of the available ports. In that list, you should see your current port (A). After that, it prints the current port and the speed used (B). Once the port is opened, you&nbsp;receive a notification that the board is initialized (C). Usually the boards report the version of Firmata that you are running. Then you&nbsp;set the sampling interval that the board uses (D). In this example, you can see that you are setting pin 13 to be an output because you&nbsp;have used the DigitalOutput component (E). Finally you&nbsp;will see that the board will send you&nbsp;a list of all the pins available and thier capabilities (F). This list contains the number of the pin and the modes in which it can be used, for example: DigitalInput, DigitalOutput, AnalogInput, AnalogOutput, and Servo.</p>
      <p><img src=\"modelica://OpenModelicaArduino/Resources/Images/ModelPlugLog-nomarkers.png\" alt=\"\" /></p>
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
      <p>OpenModelicaArduino contains a series of basic examples showing the functionality of the components. You can check the Examples under ModelPlug.Examples. Once you have learned how to use the basic components of OpenModelicaArduino, you can check the Arduino Playground to learn how to connect other sensors and actuators (<a href=\"http://playground.arduino.cc/\">http://playground.arduino.cc</a>).</p></html>", revisions = ""),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end GettingStarted;
# Microcontroller Config Instructions

## Initial Host Config Steps
### Downloading ESP-IDF Manager
1. Download the ESP-IDF package and follow the configuration steps from [here](https://developer.espressif.com/tags/esp-idf/).
2. Run the ESP-IDF installation manager (in Linux it was simply `$ eim`)
3. Select the `Easy Installation` option
4. Once the easy install is complete, follow the next steps:
<ol type="i">
   <li>Open a new IDF terminal using the icon on version manager screen or by sourcing activation script in any open terminal.</li>
   <li>The activation script can be found in `~/.espressif/tools/` folder</li>
   <li>Run 'source ~/.espressif/tools/activate_idf_v6.0.1.sh' to activate the IDF environment</li>
   <li>Run 'idf.py build' to build your project</li>
</ol>

At this point you have essentially built the ESP equivalent of a conda environment (and in a much more literal sense, a python venv). 
To build the project you need to run the EIM-IDF package manager, open an IDF terminal, and then execute the `install.sh` script that was built

### Build the Project with CLion
1. Open the ESP-IDF install manager, and select the 'Open IDF terminal' button within the appropriate installed IDF environment. Or if you don't want to do that,
open a normal terminal, and run the following command (note you'll need to replace the angle bracket string with the actual path to your ESP export.sh file)
`. <full_path_to_esp-idf_directory_containing_export.sh>`
What this does is it runs the instructions contained in export.sh in your current shell session so that environment variables needed by ESP-IDF are loaded. If you start another terminal
you will need to run that export script again.
2. Change directories to the IDF project directory<br>
    `cd $IDF_PATH/<path to directory containing project>`
3. Run the following commands in the project terminal
~~~
idf.py set-target esp32
idf.py build
~~~

### Flashing the ESP32
1. First locate the port that the ESP is connected to on your host. In Ubuntu you can run a command like `sudo dmesg | grep tty` to see tty devices. You should be looking for an entry whose description
contains something like "UART" or "cp210x" and reading the tty input. Another method is to run `echo "/dev/serial/by-id/$(ls /dev/serial/by-id)"` to print the full port path to your screen.
2. Ensure you have built the project using the steps above, and are in a terminal containing all necessary environment variables.
- You can check if your terminal has all the environment variables needed by running `printenv | grep -i ESP`. If you see entries such as `ESP_IDF_VERSION` and `ESP_CLANG_LIBS_PATH` and 
`ESP_PATH` then you do NOT need to run export.sh again.
3. In terminal currently located in the directory containing your project root CMakeLists.txt file and run the following command `idf.py -p <full_path_to_port_from_step_1_here> flash` to simply flash the 
code to the ESP device without being able to see it run. If you want to see it execute in a tty-like terminal, run the following command instead `idf.py -p <full_path_to_port_from_step_1_here> flash monitor` or 
you can simply run `idf.py -p <full_path_to_port_from_step_1_here> monitor` after flashing to the device. 
4. You should now be seeing output from your code on the screen.
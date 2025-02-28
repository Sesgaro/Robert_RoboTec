```
██████╗  ██████╗ ██████╗ ███████╗██████╗ ████████╗   ██████╗  ██████╗ ██████╗  ██████╗ ████████╗███████╗ ██████╗
██╔══██╗██╔═══██╗██╔══██╗██╔════╝██╔══██╗╚══██╔══╝   ██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔════╝
██████╔╝██║   ██║██████╔╝█████╗  ██████╔╝   ██║█████╗██████╔╝██║   ██║██████╔╝██║   ██║   ██║   █████╗  ██║     
██╔══██╗██║   ██║██╔══██╗██╔══╝  ██╔══██╗   ██║╚════╝██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██╔══╝  ██║     
██║  ██║╚██████╔╝██████╔╝███████╗██║  ██║   ██║      ██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ███████╗╚██████╗
╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝      ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝   ╚══════╝ ╚═════╝

```
# Robert
This is the official unofficial GitHub repo for the Robert (bob). All code, tests, comments, etc. go here.
Este es el repo oficial no oficial del Robert.

Python Dev
* to avoid incompatibilities with other modules installed on your pc, you must create a virtual environment.

```
py -m venv .\[folder_name]\
```

* to run this virtual environment, go to the created folder named `scripts` and run the file `activate`.

```
.\activate
```

* Now install the requirements with the following command:
```
pip install -r ./requirements.txt
```

With this, we can start, the file `UART_Test1.py` we can read 2 ESPs, this is a test for simultaneous reading of two ports when receiving data.

* In case you have problems in windows give several permissions:

```
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
```

# Update 0.2

`Pwm_and_control_Test1.py` This code is the first control connection test, controller -> Laptop/Jetson -> ESP.

The computer sends a text string each time the trigger values are updated, the ESP separates and transforms from scripts to integer and float values.

# TX1 Works with the XboxController!!
Fixed compatibility issues with Jetson and reprogrammed the file `Pwm_and_control_Test1.py` (now called `espmotors.py`) to be separated by functions, and to be invoked by `main.py`.

# Updated requirements
Due to problems with the `inputs` library we opted to change to `pygame` for reading the command.

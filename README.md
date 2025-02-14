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

* to run this virtual environment, go to the created folder named 'scripts' and run the file 'activate'.

```
.\activate
```

* Now install the requirements with the following command:
```
pip install -r ./requirements.txt
```

With this, we can start, the file 'UART_Test1.py' we can read 2 ESPs, this is a test for simultaneous reading of two ports when receiving data.

* In case you have problems in windows give several permissions:

```
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
```

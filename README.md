# Wind
Lab Project
  Verify controllability of turbine system designed elsewhere.
  
# Installation Instructions
1.  Install Link Shell Extension  http://download.cnet.com/Link-Shell-Extension-64-bit/3000-2248_4-75213087.html
        It will make you restart explorer.    
    Link Shell Manual:  http://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/
2.  Install Arduino:  Get Arduino .zip
        i.  https://www.arduino.cc/en/Main/Software    get the Windows ZIP file for non-admin install
        ii. Unzip into your “My Documents”
            Ref:  https://www.arduino.cc/en/Guide/PortableIDE  Summary follows so shouldn’t have to go here
		iii.When installed, it creates folder Arduino-1.x.x where you told it to.   This is the "root" in what follows.
3.  Browse to Arduino-1.x.x, the root, in the unzipped Arduino..   Create a new directory called “portable” alongside the others
4.  Launch the Arduino executable “arduino.exe” found in the root.  This is the IDE.
    a.  Create/Save shortcuts to taskbar and Start
    b.  IDE-File - preferences - uncheck "Show verbose output during compilation" - set Compiler warnings = none.
        Be sure to click OK even if made no changes (do not press “X”).
    c.  Close the IDE.   The defaults are now set.
5.  Browse to <root>/portable/sketchbook (Arduino-1.x.x) to create symbolic link to source
	a.  Browse to this installation folder, where this README.md file is kept.   This is <source> in what follows.
    b.  right-click on myWindCode in  <source>\portable\sketchbook and “pick link source”
    c.  right-click in <root>\portable\sketchbook.  Right-click - Drop-as - Symbolic link.
6.  Open the IDE.   File-open-browse to potWind.ino.   Click it open.
    Close all other IDE windows so IDE remembers your default choice.
     Reference: https://www.arduino.cc/en/Reference/HomePage
        In brief:  Ctrl-r to compile.   Try it now with potWind.    Ask for help if this does not work.
                   Ctrl-u to compile/upload.   Arduino needs to be connected and USB free
7.  Install CoolTerm USB monitor
    http://freeware.the-meiers.org/CoolTerm_Win.zip
    Copy CoolTerm_0.stc in this folder to the desktop for easy starting of USB
8.  Habitually calibrate your ESC:
        - Open loop on board
        - Set POT to max
        - Power the ESC.   Wait for musical phrase followed by two beeps in quick succession
        - Set POT to min.   Wait for three beeps followed by one ready beep.
    Calibration may last 100 runs or it may last 1 run.   Motion of the POT
    can undo calibration if you're not hearing all the beeps.  
    Beware of this.   It's the most common problem.
    When you start up get in the habit of spot-checking calibration:
        - Nt moves at throttle 5-15 deg
        - 80% Nt at throttle 180 deg
9.  Normal startup
    - Microcontroller powered and at 0 deg throttle (either open or closed loop); wind power supply off
    - Turn on wind power supply.    Wait and should hear 3 beeps followed by 1
10. Control checkout
    It is possible to close the loop using embedded model before firing up the turbine and damaging something.
    In potWind.ino set bareOrTest=True, recompile, reload, and run without turbine powered.
    Observe output as though running with turbine (feedback switched internally).
    The CLAW embedded model will have an erroneous Ng(throttle) characteristic because the Arduino throughput will not support a table lookup here.

# FAQ
1.  Coolterm “Port not found.”  
    Rescan ports in Coolterm-Connection-Options
    Restart Coolterm if necessary
    Turn off Arduino IDE USB monitor
2.  Suddenly bad stability.   Recalibrate ESC.
    When you start up get in the habit of spot-checking calibration:
        - Nt moves at throttle 5-15 deg
        - 80% Nt at throttle 180 deg
3.  No response.  Check for loose wires on board.
4.  Poor response.   Check for reversed wires on ESC to 3-phase. 
    Confirm direction of airflow into inlet of gas generator.
5.  Arduino IDE:  port not found following ctrl-u
    Arduino-Tools-Port-Select Arduino
6.  Simulink Wind_SIM:  Invalid setting in 'Wind_SIM/ESC_Core_Tur/Rate_Transition1' for parameter 'InitCond'
    There is a NaN in data vector.  It is probably the first row.
    Open MOD.source file and delete offending row.   Resave it and rerun Wind_SIM.
7.  Arduino IDE:  When compiling, "The system cannot find the path specified."   This means the IDE is opened in the wrong folder.    Start another IDE session by Arduino - File - Open or Open Recent. 

8.  Simulink Wind_SIM:  Multiple messages running Simulink Wind_SIM:   "x-values must be increasing."  or "Unsupported input format." There is data corruption in DS.V, easily fixed by hand.  Open the csv file.  Plot "time" all by itself with no x-values using excel plot.    See dropout.   Browse to row indicated by x-param.
9.  Simulink Wind_SIM:  Transient truncated Simulink Wind_SIM running with vectors.   There are data dropouts in csv file.  Browse in csv file to the truncated time and delete bad rows of data.
10. Simulink Wind_SIM:  There is an apparent mismatch between Nts and Nt_ref.  You are probably running in "test" mode.    Look at Figure 2 for these.
11. Arduino IDE:  When compiling, "The systsem cannot find the path specified."  The  means the IDE is opened in the wrong folder.   Start another IDE session by using IDE Arduino - File - Open - or Open Recent.
12. Simulink Wind_SIM:   "Reference to non-existent field 'tFinal'."  Cannot open the csv file.  Probably named badly.
13. Simulink Wind_SIM:  "Error du to multiple causes."  "Reference to non-existent field 'vpot'".  You have "CALIBRATING" set in myWindCode.ino.   You can re-run or rename vf2v to vpot in csv file and ignore it but this will cause initialization to go badly.   So write the real vpot into csv file as constant.
14.  Simulink Wind_SIM:  Why does Nts not equal NtsD?   Ng in the Simulink model is assumed to follow nominal trend, so same software logic can be used all units.   The real Ng is not nominal and will differ.

# Expected Results
Set "#define VECTOR" in potWind.ino
Recompile and upload
Select open or close loop mode as desired on board
Start CoolTerm data save
Press pushbutton on board to begin defined vector test
Stop CoolTerm data save when done
Compare to Saves/v<your kit #>_2017



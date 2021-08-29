# 2019_vision_cpp

## Building externally on Windows

### Setup
* Install the Raspbian compiler from https://github.com/wpilibsuite/raspbian-toolchain/releases (use **v1.2.0** -- v1.3.0 doesn't work)
* Install GNU Make from http://gnuwin32.sourceforge.net/packages/make.htm (download the complete package without sources)
* Add the bin folder for both the Raspbian compiler and GNU Make to your Path environment variable

### Building
* Open Command Prompt
* Navigate to the directory this repository is in
* Run "make"

### Deploying
* Open the Raspberry Pi web dashboard in a web browser
* Select "Writable" in the top ribbon
* Go to the Application tab (located in the left menu)
* Click "Choose File", select the multiCameraServerExample file, and click "Open"
* Click "Save"

### Executing (if automatic start doesn't occur)
* Open PuTTY
* Connect to the Raspberry Pi using SSH and log in
* Run "./runInteractive" in /home/pi or "sudo svc -t /service/camera" to restart service

### Monitoring/Debugging
Console output can be captured through PuTTY or the Vision Status tab in the Raspberry Pi web dashboard.


## Building locally on the Raspberry Pi
* Run "make"
* Run "make install" (replaces /home/pi/runCamera)
* Run "./runInteractive" in /home/pi or "sudo svc -t /service/camera" to restart service

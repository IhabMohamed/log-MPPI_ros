@echo off
setlocal

REM this batch script uses taskkill to terminate processes by executable names
REM only run this script when there is no rostopic or rosbag running
for /f "tokens=2" %%P in ('
  tasklist /fi "Imagename eq rostopic.*" /fo list ^| findstr /b "PID:"
') do (
  echo Error^! Please run this script without any running instance of rostopic^!
  exit /b 1
)
for /f "tokens=2" %%P in ('
  tasklist /fi "Imagename eq rosbag.*" /fo list ^| findstr /b "PID:"
') do (
  echo Error^! Please run this script without any running instance of rosbag^!
  exit /b 1
)

set TEMPDIR="%temp%\jackal.calibrate_compass"
if exist %TEMPDIR% (
  if exist %TEMPDIR%\nul (
    rd /s /q %TEMPDIR%
  )
)
mkdir %TEMPDIR%
set BAG_FILE="%TEMPDIR%\imu_record.bag"
set CAL_FILE="%TEMPDIR%\mag_config.yaml"
set CAL_FINAL_PATH="%ROS_ETC_DIR%\jackal_base"
set DURATION=60

call rostopic list >nul
if not "%errorlevel%" == "0" (
  echo ROS appears not to be running. Please start ROS service^!
  exit /b 1
)

call rospack find rosbag >nul
if not "%errorlevel%" == "0" (
  echo Unable to find rosbag record. Is the ROS environment set up correctly?
  exit /b 1
)

start rosbag record /tf /imu/rpy/raw /imu/data_raw /imu/mag -O %BAG_FILE% --duration %DURATION%
echo Started rosbag record, duration %DURATION% seconds

start rostopic pub /cmd_vel geometry_msgs/Twist "{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}" -r 15 >nul
echo Started motion commands

REM wait for 2 seconds
timeout /t 2 /nobreak >nul

echo Test underway.

REM wait for %DURATION%
timeout /t %DURATION% /nobreak

echo Shutting down motion command publisher.
for /f "tokens=2" %%P in ('
  tasklist /fi "Imagename eq rostopic*" /fo list ^| findstr /b "PID:"
') do (
  taskkill /f /t /pid %%P
)

REM wait for 2 seconds
timeout /t 2 /nobreak >nul

echo Waiting for rosbag to shut down.

REM wait for 2 seconds
timeout /t 2 /nobreak >nul

REM terminate any remaining rosbag process
for /f "tokens=2" %%P in ('
  tasklist /fi "Imagename eq rosbag*" /fo list ^| findstr /b "PID:"
') do (
  taskkill /f /t /pid %%P
)

echo Computing magnetic calibration.
call rosrun jackal_base compute_calibration %BAG_FILE% %CAL_FILE% >%temp%\compute_output.log 2>&1
if not "%errorlevel%" == "0" (
  echo Unable to compute calibration from recorded bag.
  echo Output in %temp%\compute_output.log
  exit /b 1
)

(set _RESPONSE=)
if exist %CAL_FILE% (
  echo Calibration generated in %CAL_FILE%.
  set /p _RESPONSE=Copy calibration to %CAL_FINAL_PATH%? [Y/n] || set response=Y
)

if defined _RESPONSE (
  if "%_RESPONSE:~0,1%" == "n" (
    echo Skipping.
  ) else (
    if "%_RESPONSE:~0,1%" == "N" (
      echo Skipping.
    ) else (
      mkdir %CAL_FINAL_PATH%
      copy %CAL_FILE% %CAL_FINAL_PATH%
      echo Restart ROS service to begin using saved calibration.
    )
  )
)

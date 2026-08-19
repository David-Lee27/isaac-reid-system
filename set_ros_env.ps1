# Run this once per new PowerShell session, BEFORE launching Isaac Sim or
# unified_tracking.py, to set up ROS2 discovery with WSL's current IP
# (which can change across reboots since we're using NAT networking mode).
#
# Usage:
#   . C:\isaacsim\projects\surveillance-proj\set_ros_env.ps1
# (note the leading ". " - this "dot-sources" it so the env vars persist
# in your current session, rather than just setting them in a subprocess)

$wslIp = (wsl hostname -I).Trim().Split(" ")[0]
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = "SUBNET"
$env:ROS_STATIC_PEERS = $wslIp

Write-Host "ROS2 discovery configured. WSL IP detected: $wslIp"

# Make sure to enable execution via Powershell with administrator privileges: Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope LocalMachine
$env:MUJOCO_DYNAMIC_LINK_DIR=(Resolve-Path "mujoco-3.3.7/lib").Path
$env:PATH=(Resolve-Path "mujoco-3.3.7/bin").Path

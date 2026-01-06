$VERSION = "3.3.7"

$url  = "https://github.com/google-deepmind/mujoco/releases/download/$VERSION/mujoco-$VERSION-windows-x86_64.zip"
$file = "mujoco-$VERSION-windows-x86_64.zip"
$dest = "mujoco-$VERSION"

# Download
Invoke-WebRequest -Uri $url -OutFile $file

# Ensure destination folder exists
New-Item -ItemType Directory -Path $dest -Force | Out-Null

# Extract into mujoco-3.3.7
Expand-Archive -Path $file -DestinationPath $dest -Force

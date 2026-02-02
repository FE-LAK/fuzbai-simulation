# Builds the simulation-app / competition application.

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# ----------------------------
# Prepare variables
# ----------------------------
$CWD = Get-Location
$OUTPUT = "target/BUILD_OUTPUT_HERE"
$OUTPUT_APP = Join-Path $OUTPUT "simulation-app"
$OUTPUT_PYTHON = Join-Path $OUTPUT "python"
$CLEAN_DIRS = @("target")

# Default values
$APP = "y"
$PYTHON = "n"
$CLEAN = "n"

# ----------------------------
# Parse CLI arguments
# ----------------------------
foreach ($arg in $args) {
    switch -Regex ($arg) {
        '^--app=(.+)$'     { $APP = $Matches[1] }
        '^--python=(.+)$'  { $PYTHON = $Matches[1] }
        '^--clean$'        { $CLEAN = "y" }
        '^--help' {
            Write-Host @"
Helper script for building this project.

Options:
    --app=y/n       build the simulation (competition) application.
    --python=y/n    build python bindings of the simulation (without competition application).
    --help          opens this help menu.
    --clean         deletes directories: $($CLEAN_DIRS -join ", ")
"@
            exit 0
        }
        default {
            Write-Error "Unknown option: $arg"
            exit 1
        }
    }
}

# ----------------------------
# Clean
# ----------------------------
if ($CLEAN -eq "y") {
    foreach ($dir in $CLEAN_DIRS) {
        if (Test-Path $dir) {
            Write-Host "Deleting $dir"
            Remove-Item -Recurse -Force $dir
        }
    }
    exit 0
}

# ----------------------------
# Set MuJoCo variables
# ----------------------------
. ./setup_windows.ps1

# ----------------------------
# Create output directory
# ----------------------------
New-Item -ItemType Directory -Path $OUTPUT -Force | Out-Null

# ----------------------------
# Build application
# ----------------------------
if ($APP -eq "y") {
    $env:RUSTFLAGS="-C target-feature=+crt-static"
    cargo build --release -p simulation-app --locked

    New-Item -ItemType Directory -Path $OUTPUT_APP -Force | Out-Null

    Copy-Item "target/release/simulation-app.exe" $OUTPUT_APP -Force
    Copy-Item "simulation-app/www" $OUTPUT_APP -Recurse -Force

    # Platform-specific MuJoCo library
    if (Test-Path "mujoco-3.3.7/lib/libmujoco.so") {
        # Linux (unlikely on Windows, but kept for parity)
        Copy-Item "mujoco-3.3.7/lib/libmujoco*" $OUTPUT_APP -Force
    }
    else {
        # Windows
        Copy-Item "mujoco-3.3.7/bin/mujoco.dll" $OUTPUT_APP -Force
    }
}

# ----------------------------
# Build Python bindings
# ----------------------------
if ($PYTHON -eq "y") {
    Push-Location "simulation"
    $env:RUSTFLAGS="-C target-feature=+crt-static"
    maturin build --release --locked
    Pop-Location

    New-Item -ItemType Directory -Path $OUTPUT_PYTHON -Force | Out-Null
    Copy-Item "target/wheels/*" $OUTPUT_PYTHON -Recurse -Force
}

# ----------------------------
# Done
# ----------------------------
Write-Host "============================="
Write-Host "Finished! Saved in $OUTPUT"
Write-Host "============================="

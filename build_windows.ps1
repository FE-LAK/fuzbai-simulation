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
$OUTPUT_DOC = Join-Path $OUTPUT "doc"
$CLEAN_DIRS = @("target")

# Default values
$APP = "y"
$PYTHON = "n"
$DOC = "n"
$LICENSES = "n"
$CLEAN = "n"

# ----------------------------
# Parse CLI arguments
# ----------------------------
foreach ($arg in $args) {
    switch -Regex ($arg) {
        '^--app=(.+)$'     { $APP = $Matches[1] }
        '^--python=(.+)$'  { $PYTHON = $Matches[1] }
        '^--doc=(.+)$'     { $DOC = $Matches[1] }
        '^--licenses=(.+)$'  { $LICENSES = $Matches[1] }
        '^--clean$'        { $CLEAN = "y" }
        '^--help' {
            Write-Host @"
Helper script for building this project.

Options:
    --app=y/n       [default=y] build the simulation (competition) application.
    --python=y/n    [default=n] build python bindings of the simulation (without competition application).
    --doc=y/n       [default=n] build documentation of the simulation (including the Python bindings).
    --licenses=y/n  [default=n] generate a licenses report (in HTML form) of embeded libraries.
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
# Build documentation
# ----------------------------
if ($DOC -eq "y") {
    $env:DOCS_RS = "y"
    cargo doc -p fuzbai_simulator --no-deps --document-private-items --features python-bindings --locked

    New-Item -ItemType Directory -Path $OUTPUT_DOC -Force | Out-Null
    Copy-Item "target/doc/*" $OUTPUT_DOC -Recurse -Force

    # Create a simple Internet shortcut (`.url`) that points to the generated docs index
    $docIndex = Join-Path $OUTPUT 'doc\fuzbai_simulator\index.html'
    $resolved = (Resolve-Path $docIndex).Path
    $url = 'file:///' + ($resolved -replace '\\','/')
    $urlPath = Join-Path $OUTPUT 'DOCUMENTATION.url'
    Set-Content -Path $urlPath -Value ("[InternetShortcut]`nURL=$url") -Encoding ASCII
}

# ----------------------------
# Generate licenses
# ----------------------------
if ($LICENSES -eq "y") {
    # Setup cargo-about
    cargo install cargo-about --locked
    # Generate licenses file
    cargo about generate about.hbs --features python-bindings -o $OUTPUT/THIRD_PARTY_NOTICES.html
}

# ----------------------------
# Done
# ----------------------------
Write-Host "============================="
Write-Host "Finished! Saved in $OUTPUT"
Write-Host "============================="

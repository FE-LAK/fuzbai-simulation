---
name: clippy
description: Run cargo clippy for Rust linting. Use this when asked to lint the code or check for common mistakes.
---

# Running Clippy

Run `cargo clippy` to catch common Rust mistakes, style issues, and potential bugs.

1. Source the platform-specific setup script to set MuJoCo environment variables:
```bash
source setup_linux.sh   # or setup_macos.sh / setup_windows.ps1
```

2. Run clippy across all targets:
```bash
cargo clippy --all-targets -- -D warnings 2>&1
```

3. Verify the exit code is 0 (success). Fix any warnings before considering the work done.

> [!NOTE]
> - `--all-targets` checks lib, tests, examples, and benches.
> - `-D warnings` treats all warnings as errors so nothing is silently ignored.
> - Read `Cargo.toml` `[features]` for available features and enable them if needed.
> - Do NOT suppress clippy lints with `#[allow(...)]` without a justifying comment.

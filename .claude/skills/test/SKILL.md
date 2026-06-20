---
name: test
description: Run the Rust tests for this project. Use this when asked to run tests or verify that changes work correctly.
---

# Running Tests

1. Source the platform-specific setup script to set MuJoCo environment variables:
```bash
source setup_linux.sh   # or setup_macos.sh / setup_windows.ps1
```

2. Run all tests:
```bash
cargo test 2>&1
```

3. Verify the exit code is 0 (success).

> [!NOTE]
> - To run a single test: `cargo test <test_name>`.
> - To run in release mode add `--release`.
> - Read `Cargo.toml` `[features]` for available features and enable them if needed.

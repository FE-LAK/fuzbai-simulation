---
name: comprehensive-code-review
description: >
  Performs a thorough, multi-agent code review of the codebase.
  Use this when asked to review for bugs, memory problems, correctness issues,
  or when asked for a comprehensive code audit.
  Agents must NOT modify any code --- this is a read-only audit.
---

# Comprehensive Code Review

Perform a deep, parallelized review of the codebase by spawning **5 parallel
code-review agents**, each focused on a distinct category. Agents must **not
modify any code** --- they only report findings. Collect all results and present
a consolidated report.

## Codebase Overview

This is a Rust workspace containing a real-time robotic foosball (table football)
simulation built on MuJoCo physics. Key technology:

- **MuJoCo bindings** via the `mujoco-rs` crate (FFI wrapper around the C library)
- **actix-web** HTTP servers for agent communication and management
- **PyO3** for Python interop (enum serialization/deserialization)
- **Real-time physics loop** with sub-step collision tracking and motor control
- **Shared state** across simulation thread, UI thread, and HTTP worker threads

Each agent must first explore the repository structure and source files to
understand the codebase before beginning its review. Do not hardcode file paths
in the agent prompts --- instruct agents to discover the relevant files themselves.

## Review Categories

### 1. Unsafe Code, FFI & Memory Safety
- All `unsafe` blocks and raw pointer usage
- Unchecked coercions (`transmute`, `from_raw_parts`, etc.), especially from external input
- FFI boundaries: panics/unwinds through C frames, `extern "C-unwind"` correctness
- Callback safety (error handlers, control callbacks registered with native libraries)
- Static lifetime patterns and dangling reference risks

### 2. Concurrency & Synchronization
- Deadlock potential from nested lock acquisitions (verify lock ordering across all threads)
- Race conditions and TOCTOU bugs on shared state
- Mutex poisoning and panic-safety patterns
- Contention on hot-path locks
- Spin-wait / busy-loop correctness

### 3. Logic Bugs & Algorithmic Correctness
- Loop control flow (`break`/`continue`) that may short-circuit iteration incorrectly
- Mismatched documentation vs. implementation
- Index mapping correctness across coordinate systems or data transforms
- Edge cases in arithmetic (division by zero, overflow, NaN/Inf propagation)
- Off-by-one errors

### 4. Dependency Deep Inspection
- Verify every call to external crate APIs against their source code
  (find dependency sources in the cargo registry cache or target directory)
- Check that handles, metadata, or cached state remain valid across resets/reloads
- Verify FFI function signatures, array sizes, and calling conventions in the bindings
- Read the dependency wrapper source extensively for bugs, memory problems,
  undefined behavior, or unsound FFI --- specifically in the code paths this
  repository exercises
- Trace wrappers down to their underlying native/C calls and verify correctness
- Check whether any usage pattern could trigger a latent bug in the dependency

### 5. HTTP/API Robustness & Resource Management
- Route/handler registration completeness (all defined handlers must be wired up)
- Input validation on payloads
- Serialization/deserialization correctness (double-serialization, type mismatches)
- Timing or frequency calculations that can produce Inf/NaN
- Shutdown correctness and resource cleanup

## Output Quality Rules

Agents must produce clean, professional output:
- Use only ASCII characters. Do not use Unicode dashes, bullets, arrows, or
  other non-ASCII symbols.
- Do not use emoji.
- Use `---` instead of em-dashes, `-` for list bullets, `->` for arrows.
- Write in clear, direct technical English.

## Execution

1. Spawn all 5 agents **in parallel** using the `task` tool with
   `agent_type: "code-review"` and `model: "claude-opus-4.6"`.
2. Each agent's prompt must:
   - Include the specific review category and focus areas from above.
   - Instruct the agent to explore the repository structure to find relevant
     source files (do not list file paths for it).
   - Explicitly state: **"Do NOT modify any files. Report findings only."**
   - Include the output quality rules above.
3. Wait for all agents to complete.
4. Consolidate findings into a single report, grouped by severity:
   Critical > High > Medium > Low.
5. For each finding, include: file path, line number, severity, problem
   description, evidence, and suggested fix.
6. Omit categories where no issues were found, with a brief
   "no issues found" note.

# Coding Style

## Meta

- **Self-updating rules**: When the user gives explicit, major style feedback during a session
  (e.g., correcting how code should be formatted or structured), update the relevant rule file
  in `.claude/rules/` immediately so the rule is captured permanently and applies to all future
  sessions and subagents.

## General

- Use only ASCII characters in all code, comments, and strings.
- Do not fix pre-existing style issues in committed code. Only flag and fix style problems
  in new, uncommitted changes.
- Do not make unnecessary edits to already compiling code. If the requested task is satisfied
  and the code compiles, avoid extra refactors or cleanup not required for correctness.

## Code Minimality

- **DRY**: Every piece of logic must have a single representation. If two functions share the
  same body differing only in a type or minor parameter, make the function generic or extract
  a shared helper. Never duplicate an implementation.
- **YAGNI**: Only implement what is explicitly asked for. Do not add speculative parameters,
  extra overloads, "just in case" variants, or helper types that no caller currently uses.
- **KISS**: Prefer the simplest solution that fully solves the problem. A longer but readable
  solution is better than a short but tricky one.
- **Generics over duplication**: When two or more functions share identical logic but differ
  only in a numeric type, make the function generic rather than writing separate variants.
- **No redundant wrappers**: Do not create a function that solely delegates to another function
  of the same name with no added logic.
- **No unused parameters**: Every function parameter must be used.

## Imports

Order import lines by length, longest line first. Items that belong together (e.g., from the
same parent module or ecosystem) should be grouped together in adjacent lines. When adjacent
import groups have fundamentally different parent modules (e.g., a web framework vs. a
serialization library, or std vs. tokio), separate them with an empty line.

## Comments

- Use `///` doc comments for public APIs, types, and constants.
- Use `/* */` block comments for major section headers.
- Use `//` inline comments sparingly for non-obvious logic.
- Keep comments concise and technical. Use backticks for cross-references to types, functions,
  or constants.
- Use `/// # Safety` doc sections on `unsafe fn` declarations to document caller obligations.
- Use `// SAFETY:` inline comments on `unsafe {}` blocks to explain why the block is sound.
  Do not duplicate a `// SAFETY:` comment already stated earlier in the same function or
  impl scope; only add a new one when the reasoning differs.

## Code Style

- **Opening brace**: When a function signature fits on one line, place `{` on the same line
  (K&R style). When parameters are split across multiple lines, place `{` on its own line
  (Allman style) so the body is visually separated from the signature.
- **No inlined function bodies**: Never write a function body on the same line as its
  signature (e.g., `fn foo(...) -> T { ... }`). Always expand the body to its own indented
  lines, even for trivial one-liners such as builder setters.
- **References for non-trivial types**: When binding a value larger than a pointer/reference
  (e.g., an array, struct, or tuple larger than 8 bytes on a 64-bit target), bind it by
  reference (`let x = &expr`) rather than copying it, unless a copy is explicitly required.
- **`if let` over `matches!`**: In conditionals, prefer `if let Some(x) = expr` over
  `if matches!(expr, Some(x))`. `matches!` is acceptable inside `assert!` in tests.
- **`debug_assert!` policy**: Use `debug_assert!` for invariants guaranteed by the
  implementation that would only fail due to a bug in the code itself. Use `assert!` or
  `Result`-returning error handling for conditions that can fail in correct code (e.g.,
  allocation failures, user-facing precondition violations).

## Testing

- Do not write any tests in the crates of this repository (`fuzbai-simulation/**`). Do not add
  new test modules, `#[test]` functions, integration tests, or doctests, and do not extend
  existing ones, unless the user explicitly requests it.

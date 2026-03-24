# Coding Style

## Imports

### Line Length

Order import lines by length, longest line first.

### Grouping

Items that belong together (e.g., from the same parent module or ecosystem)
should be grouped together in adjacent lines.

### Spacing

When adjacent import groups have fundamentally different parent modules
(e.g., a web framework vs. a serialization library, or std vs. tokio),
separate them with an empty line.

## Comments

- Use `///` doc comments for public APIs, types, and constants.
- Use `/* */` block comments for major section headers.
- Use `//` inline comments sparingly for non-obvious logic.
- Keep comments concise and technical. Use backticks for cross-references
  to types, functions, or constants.

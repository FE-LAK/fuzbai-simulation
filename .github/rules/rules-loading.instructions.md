# Rules Loading

## Main Agent

At the start of each user request, read all files in `.github/rules/` using the
`view` tool and follow the instructions found there. Briefly acknowledge which
rules you loaded before proceeding with the user's request.

After each `ask_user` call or new user prompt, re-read or recall the rules and
continue to follow them. If you cannot re-read the files, summarize the rules
you remember and continue to apply them.

## Subagents

When spawning subagents (via the `task` tool), instruct each subagent to read
all files in `.github/rules/` and `.github/skills/` before starting its work.
Subagents do not automatically inherit rules or skills from the repository ---
they must discover and read them on their own.

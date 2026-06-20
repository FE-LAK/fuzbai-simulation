# Disallowed Commands

Commands that must never be executed. Refuse or propose alternatives if a request
would require running any of these.

## Destructive Git Operations

- `git reset --hard` / `git clean` -- wipe working files.

## Network & Remote Access

- `curl`, `wget`, `scp`, `rsync`, `ssh`, `ftp`, or any tool for downloading,
  uploading, or connecting to external hosts.
- Package managers with remote access (`npm install`, `pip install`, etc.) when
  they would fetch from external repositories. Tell the user what to run instead.

## Server / Listener Binding

Any HTTP server, TCP listener, or similar network service **must bind exclusively
to `127.0.0.1` (localhost)**. Never bind to `0.0.0.0`, `::`, or any interface
that exposes the port to the network.

Correct example: `python -m http.server 8080 --bind 127.0.0.1`

## Privilege Escalation & System Modifications

- `sudo`, `su`, or any command that requests elevated privileges.
- `chmod`, `chown`, `passwd`, `groupadd`, `useradd`.
- `apt-get`, `yum`, `brew`, `pacman` -- installing or removing system packages.
- `mkfs`, `fdisk`, `parted`, `mount`, `umount`.

## File & Data Destruction

- `rm -rf` or any command that irreversibly deletes large directories or critical files.
- `dd`, `shred`, `mkfs`, or other low-level data wipe utilities.
- `find / -exec ... \;` with destructive actions traversing the whole filesystem.

## Build & Execution Risks

- Starting background services using shell-level mechanisms such as `nohup ... &`,
  `setsid`, `disown`, `systemctl start`, `service`, or `docker run`. Use the `Bash`
  tool with `run_in_background: true` when a background process is genuinely required.

## Miscellaneous

- Any command that sends data or credentials to external systems.
- `echo $SECRET` or reading environment variables containing secrets.

When in doubt, describe the action to the user rather than executing a potentially
harmful command.

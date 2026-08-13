# Security Policy

## Reporting a security problem

Please do **not** publish exploit details, malicious payloads, credentials, or private GPS data in a public Issue.

If GitHub's **Report a vulnerability** / private vulnerability reporting option is enabled for this repository, use that private channel for security-sensitive reports.

For ordinary bugs that are not security-sensitive, use a normal GitHub Issue.

## Release safety

Official executable downloads should come only from Releases published in this repository by the maintainer. A fork, Pull Request, comment, or user attachment is not an official build.

Before publishing a new executable, the maintainer should build it from the reviewed `main` branch on a trusted machine and scan the resulting file with the operating system's security tools.

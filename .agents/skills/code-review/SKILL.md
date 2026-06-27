---
name: code-review
description: Review staged/unstaged changes plus commits ahead of origin/main. Produces an HTML report in the OS temp dir, then grills the user with docs on findings.
---

# Code Review

Review the working tree (tracked + untracked changes) and commits ahead of `origin/main`. The output is a self-contained HTML report in the OS temp dir, followed by a grilling session where findings can crystallize into ADRs or glossary updates.

## Process

### 1. Gather context

Run these in parallel:

- `git --no-pager diff HEAD` — unstaged + staged tracked changes
- `git --no-pager diff --cached` — staged-only changes (to distinguish from unstaged)
- `git --no-pager log origin/main..HEAD --oneline --no-decorate` — commits ahead of origin/main
- `git --no-pager log origin/main..HEAD --stat` — files touched in those commits
- `git --no-pager log origin/main..HEAD -p` — full diff of commits ahead
- `ls-files --others --exclude-standard` — untracked files

Read `CONTEXT.md` and any ADRs in `docs/adr/` to ground the review in the project's domain language and known architectural decisions.

### 2. Plan the review

From the diff scope, decide which review dimensions to delegate. Don't spawn an agent for dimensions that have no relevant changes (e.g. no new unsafe blocks → skip safety agent). The available dimensions:

| Dimension | Focus | When to skip |
|---|---|---|
| **Safety & correctness** | `unsafe` blocks, panic paths, `unwrap`/`expect`, lock ordering, interrupt safety, `static mut` | No new unsafe/fallible paths |
| **Embedded concerns** | Stack usage, allocation patterns, `no_std` compliance, peripheral conflicts, interrupt priorities, DMA hazards | Only pure-logic changes |
| **Architecture & depth** | Module seams, shallowness, leakage, leverage, locality (using `improve-codebase-architecture` vocabulary) | Trivial refactors with no structural change |
| **Style & idioms** | Naming conventions, doc comments, clippy adherence, `defmt::Format` derives, pattern consistency | Cosmetic-only changes (the agent still runs but fast-paths) |
| **Domain model** | Consistency with `CONTEXT.md` terms, new concepts needing glossary entries, ADR conflicts | No new domain concepts introduced |
| **Testability** | Test seams, untestable paths, missing test coverage for new logic, test quality | No new logic or only dead-code removal |

If the diff is small (≤3 files, ≤100 lines), skip the sub-agent fan-out and review directly. For anything larger, spawn sub-agents.

### 3. Spawn sub-agents (parallel)

For each applicable dimension, spawn one agent. Each gets:
- The full diff context (commits + working tree)
- The relevant subset of `CONTEXT.md` terms
- Dimension-specific instructions (see [DIMENSIONS.md](DIMENSIONS.md))
- An output schema: `[{file, line, severity, category, title, detail}]`

Agents run in parallel. Collect all findings when done.

### 4. Synthesize findings into an HTML report

Write a self-contained HTML file to the OS temp dir. Resolve the temp dir from `$TMPDIR` or `$TEMP`, falling back to `/tmp`, and write to `<tmpdir>/code-review-<timestamp>.html`. Open it via `xdg-open <path>` (Linux), `open <path>` (macOS), or `start <path>` (Windows).

See [HTML-REPORT.md](HTML-REPORT.md) for the scaffold and styling.

The report structure:

- **Header**: branch name, review scope (N commits ahead + working tree), timestamp, total finding count
- **Summary cards**: one per dimension, showing count of findings by severity (🔴 Critical / 🟡 Warning / 🔵 Note)
- **Findings table**: sortable, filterable table with columns: File, Line, Severity, Category, Title
- **Detail panels**: click a finding → expand inline showing the detail explanation and the relevant code snippet
- **Top issues**: the 3 most actionable findings, with concrete fix suggestions

### 5. Grill with docs

After the user has seen the report, drop into a grilling conversation. Walk findings by severity (critical first). For each finding the user wants to act on:

- **Architectural finding?** Offer to record as an ADR: "Want me to record this as ADR-NNNN in `docs/adr/`?"
- **New domain concept surfaced?** Offer to add it to `CONTEXT.md`: "This introduces the concept of X — want me to add it to the glossary?"
- **Pattern to encode?** Offer to encode it in a testing decision or ADR.

Use the grilling discipline from the `grill-me` skill: one question at a time, walk each branch, resolve dependencies before moving on.

Ask: "Which findings should we act on first?"

---
name: to-issues
description: Break a plan, spec, or PRD into independently-grabbable implementation slices written as markdown files in docs/issues/ using tracer-bullet vertical slices.
disable-model-invocation: true
---

# To Issues

Break a plan into independently-grabbable implementation slices using vertical slices (tracer bullets). Each slice becomes a markdown file in `docs/issues/`.

## File structure

```
docs/
├── prd/
│   └── mobile-check-deposit.md
├── issues/
│   ├── 0001-add-check-image-capture.md
│   ├── 0002-submit-deposit-to-backend.md
│   └── 0003-display-deposit-status.md
└── adr/
```

Create `docs/issues/` lazily — only when the first issue is written.

## Process

### 1. Gather context

Work from whatever is already in the conversation context. If the user references a PRD file, read it and use its contents.

### 2. Explore the codebase (optional)

If you have not already explored the codebase, do so to understand the current state of the code. Issue titles and descriptions should use the project's domain glossary vocabulary, and respect ADRs in the area you're touching.

Look for opportunities to prefactor the code to make the implementation easier. "Make the change easy, then make the easy change."

### 3. Draft vertical slices

Break the plan into **tracer bullet** slices. Each slice is a thin vertical cut that cuts through ALL integration layers end-to-end, NOT a horizontal slice of one layer.

<vertical-slice-rules>

- Each slice delivers a narrow but COMPLETE path through every layer (schema, API, UI, tests)
- A completed slice is demoable or verifiable on its own
- Any prefactoring should be done first

</vertical-slice-rules>

### 4. Quiz the user

Present the proposed breakdown as a numbered list. For each slice, show:

- **Title**: short descriptive name
- **Blocked by**: which other slices (if any) must complete first
- **User stories covered**: which user stories this addresses (if the source material has them)

Ask the user:

- Does the granularity feel right? (too coarse / too fine)
- Are the dependency relationships correct?
- Should any slices be merged or split further?

Iterate until the user approves the breakdown.

### 5. Write the slices to docs/issues/

For each approved slice, write a markdown file to `docs/issues/`. Use sequential numbering: `0001-slug.md`, `0002-slug.md`, etc. Scan the directory for the highest existing number and increment. Derive the slug from the slice title: lowercase, hyphenated.

Write files in dependency order (blockers first) so you can reference real filenames in the "Blocked by" field.

<issue-template>
## Parent

A reference to the source PRD file (e.g. `docs/prd/mobile-check-deposit.md`), if one exists. Otherwise omit this section.

## What to build

A concise description of this vertical slice. Describe the end-to-end behavior, not layer-by-layer implementation.

Avoid specific file paths or code snippets — they go stale fast. Exception: if a prototype produced a snippet that encodes a decision more precisely than prose can (state machine, reducer, schema, type shape), inline it here and note briefly that it came from a prototype. Trim to the decision-rich parts — not a working demo, just the important bits.

## Acceptance criteria

- [ ] Criterion 1
- [ ] Criterion 2
- [ ] Criterion 3

## Blocked by

- `0001-other-slice.md` (or link to the blocking file)

Or "None - can start immediately" if no blockers.

</issue-template>

Do NOT modify any parent PRD file.

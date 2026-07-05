# Review Dimension Instructions

Each dimension below is a self-contained prompt that can be handed to a sub-agent along with the diff context.

---

## Safety & Correctness

You are reviewing an embedded Rust project (no_std, embassy, RP2350). Focus only on safety and correctness.

**Check for:**

1. **Unsafe blocks** — every `unsafe { }` must be justified. Flag any that:
   - Are unnecessary (the operation has a safe equivalent)
   - Lack a `// SAFETY:` comment explaining the invariant
   - Invoke FFI or raw pointer manipulation without a clear safety argument
   - Interact with MMIO/pac without clear ownership documentation

2. **Panic paths** — the project denies `unwrap_used` and `expect_used` via clippy. Flag any remaining:
   - `unwrap()`, `expect()`, explicit `panic!()`
   - Array indexing without bounds checks (in release)
   - Division/modulo without zero-checks where the divisor is dynamic
   - `unwrap_unchecked` or similar

3. **Lock ordering** — the project has a documented lock order: Power → Calibration → Perception → Motion. Flag:
   - Mutex acquisitions that violate this order
   - Potential deadlocks from holding a lock across an `.await`
   - `CriticalSectionRawMutex` held for too long (embassy requires short critical sections)

4. **Interrupt safety** — flag:
   - Shared mutable state accessed from both interrupt and task context without proper synchronization
   - `static mut` usage (the project uses `Mutex` and `Atomic*` — flag deviations)
   - `Cell`/`RefCell` used in ways that could panic at runtime

5. **Resource ownership** — flag:
   - Multiple tasks taking ownership of the same peripheral
   - `static` peripheral handles that could cause use-after-move
   - `Send`/`Sync` violations (embassy tasks are not `Send`)

**Output format:** `[{file, line, severity: "critical"|"warning"|"note", category: "safety", title, detail}]`

Be specific — include line numbers, the unsafe/fallible construct, and a one-sentence fix suggestion.

---

## Embedded Concerns

You are reviewing an embedded Rust project targeting RP2350 (Cortex-M33, no_std, embassy). Focus only on embedded-specific concerns.

**Check for:**

1. **Stack usage** — embassy tasks have fixed stack sizes (see `CORE1_STACK` in main.rs). Flag:
   - Large stack allocations (arrays > 512 bytes on the stack)
   - Recursive functions (embedded stacks are small)
   - `#[embassy_executor::task]` functions that look stack-heavy

2. **Heap allocations** — this is a no_std project. Flag:
   - Any use of `alloc::` or `Box`, `Vec`, `String` from std
   - `heapless` types that could exceed their const-generic capacity at runtime
   - `format!()` or other allocation-requiring macros (use `defmt!` logging instead)

3. **Peripheral conflicts** — flag:
   - GPIO pins used by multiple peripherals simultaneously
   - I2C/SPI bus sharing issues
   - PIO state machine conflicts
   - DMA channel conflicts

4. **No_std compliance** — flag:
   - `use std::*` imports
   - Functions that require `std` (filesystem, networking, threading)
   - Floating-point operations in interrupt context (soft-float targets may have issues)

5. **Power & timing** — flag:
   - Busy-wait loops without yield/sleep (wastes battery)
   - Missing `embassy_time` timeouts on async operations that could hang
   - Hard-coded delays that should be derived from timing requirements

**Output format:** `[{file, line, severity: "critical"|"warning"|"note", category: "embedded", title, detail}]`

---

## Architecture & Depth

You are reviewing an embedded Rust project for architectural quality. Use the vocabulary from the project's architecture language:

- **Module**: anything with an interface and an implementation
- **Interface**: everything a caller must know (types, invariants, error modes, ordering, config)
- **Depth**: behaviour behind a small interface (deep = high leverage, shallow = interface ≈ implementation)
- **Seam**: where an interface lives; a place behaviour can be altered without editing in place
- **Adapter**: a concrete thing satisfying an interface at a seam
- **Leverage**: what callers get from depth
- **Locality**: change/bugs/knowledge concentrated in one place

**Check for:**

1. **Shallow modules** — where the interface nearly matches the implementation. Apply the deletion test: if you deleted this module, would complexity vanish (pass-through) or reappear across N callers (earning its keep)?

2. **Seam leakage** — where implementation details of one module leak through the interface into another. Flag: modules importing internal types from sibling modules, callers reaching behind the interface.

3. **Tightly-coupled modules** — modules that change together, are tested together, and can't be understood separately. Flag when a change in one module forces predictable changes in another.

4. **One-adapter seams** — seams with only one adapter are hypothetical. Flag these. Two+ adapters justify the seam.

5. **Lost locality** — logic for one concern spread across multiple modules. Pure functions extracted just for testability but the real bugs hide in how they're called.

6. **ADR conflicts** — check findings against the ADRs in `docs/adr/`. Flag contradictions with existing decisions.

**Output format:** `[{file, line, severity: "critical"|"warning"|"note", category: "architecture", title, detail}]`

Keep it concrete — name specific modules, interfaces, and seams. Use the project's domain vocabulary from CONTEXT.md.

---

## Style & Idioms

You are reviewing an embedded Rust project for code style and idiom consistency. Do not flag trivial formatting issues (rustfmt handles those). Focus on meaningful style concerns.

**Check for:**

1. **Naming** — flag:
   - Acronyms not consistently cased (e.g. `Imu` vs `IMU` — look at the project convention)
   - Generic names in specific contexts (`data`, `info`, `result`, `tmp`)
   - Module/file names that don't match their primary type/concern
   - No new acronyms introduced without glossary entries

2. **Documentation** — flag:
   - Public items without doc comments (`///` or `//!`)
   - Doc comments that are just the item name repeated
   - Missing `# Safety` sections on unsafe functions
   - Missing `# Panics` sections on functions that can panic
   - Missing `# Errors` sections on fallible functions

3. **Pattern consistency** — flag deviations from established project patterns:
   - The project uses `defmt::Format` derive on public types — flag missing derives
   - The project uses `#[embassy_executor::task]` for async tasks — flag raw async fn spawns
   - The project uses `CriticalSectionRawMutex` + `Mutex` for shared state — flag other synchronization
   - The project uses `Channel` from `embassy_sync` — flag other channel types

4. **Clippy** — flag:
   - Patterns that `clippy::pedantic` or `clippy::nursery` would catch (the project enables these)
   - `as` casts that could silently truncate (use `.into()` or `try_into()`)
   - Unnecessary clones or copies
   - `let _ = ...` where the result should be handled

5. **Error handling** — flag:
   - Swallowed errors (the project denies `unwrap`/`expect`)
   - Missing `#[must_use]` on result-returning functions
   - Inconsistent error type usage

**Output format:** `[{file, line, severity: "critical"|"warning"|"note", category: "style", title, detail}]`

---

## Domain Model

You are reviewing an embedded robot project for domain model consistency. The project maintains a domain glossary in `CONTEXT.md` with carefully chosen terms and "Avoid" aliases.

**Check for:**

1. **Glossary consistency** — flag any use of terms listed as "Avoid" in CONTEXT.md (e.g. "scan" instead of "sweep", "heading error" instead of "accumulated drift", "step" instead of "leg").

2. **New concepts** — flag any new domain concept introduced in the changes that doesn't have a glossary entry yet. Propose a definition and canonical name. Check if the concept already exists under a different name.

3. **Overloaded terms** — flag when an existing glossary term is being used inconsistently (e.g. "gap" used to mean something different from the glossary definition).

4. **ADR references** — flag when the code references a concept decided in an ADR but uses different terminology than the ADR itself.

5. **Missing context** — flag when a new module or type name introduces a concept that should be in the glossary but isn't. E.g. a new module called `obstacle_fusion` should have "Obstacle Fusion" in the glossary.

**Output format:** `[{file, line, severity: "critical"|"warning"|"note", category: "domain", title, detail}]`

For each finding, propose the exact glossary entry text or the term that should be used instead.

---

## Testability

You are reviewing an embedded Rust project for testability. The project targets no_std embassy on RP2350, so on-target testing is limited, but module-level testing is possible where seams exist.

**Check for:**

1. **Untestable paths** — flag new logic that has no seams for testing:
   - Logic embedded directly in `#[embassy_executor::task]` functions (can't be called from tests)
   - Logic that takes concrete peripheral types instead of traits
   - Time-dependent logic without a clock abstraction

2. **Missing tests** — flag:
   - New pure functions or modules with no corresponding tests
   - Previous tests that should be updated for changed behavior but weren't
   - Boundary conditions not covered by existing tests

3. **Test quality** — flag:
   - Tests that test implementation details rather than external behavior
   - Tests with no assertions (or assertions that can't fail)
   - Tests that depend on timing (sleep/delay) rather than determinism

4. **Test seams** — flag opportunities:
   - Where extracting an interface would enable testing (one real adapter on-target, one mock adapter in tests)
   - Where existing seams could be used but aren't (e.g. a trait bound that could accept a test double)

5. **Hardware coupling** — flag:
   - New code that directly accesses hardware registers instead of going through HAL abstractions
   - Code that assumes specific hardware configuration without making it injectable

**Output format:** `[{file, line, severity: "critical"|"warning"|"note", category: "testability", title, detail}]`

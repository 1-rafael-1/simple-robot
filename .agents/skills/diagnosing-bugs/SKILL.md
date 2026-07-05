---
name: diagnosing-bugs
description: Disciplined diagnosis loop for hard bugs. Use when the user reports a defect, says "diagnose"/"debug this", describes something broken/throwing/failing/slow, or wants to investigate a misbehavior. Grounds investigation in ADRs and CONTEXT.md so the gap between intended design and observed behavior is always clear. Handles code bugs, sensor-drift bugs, event-misfire bugs, and physical-world noise through a dedicated HITL iteration loop.
---

# Diagnosing Bugs

A disciplined loop for hard bugs. **Skip phases only when explicitly justified.** If you catch yourself jumping ahead (fixing before understanding, hypothesising before tracing), stop — that's the exact failure mode this skill prevents.

The skill works in two tracks:

- **Track A — Static analysis first.** Read the code, trace the data flow, find where it diverges from the aligned intended behaviour. Many bugs are visible in the code once you know what you're looking for. If found, grill fix avenues and execute.
- **Track B — Dynamic debugging.** If static analysis doesn't reveal the root cause (code looks correct, bug is in sensor data / timing / events / hardware / physical-world noise), fall into a feedback loop — HITL for embedded systems, automated loops for server-side code.

---

## Phase 0 — Align on the gap (system understanding via grilling + code reading)

This is the foundation. Before diagnosing, align on **what is happening** vs **what should be happening**, and achieve enough system understanding to trace the relevant code paths. Don't skip this.

### 0.1 Capture the symptom

Ask the user, one question at a time:

1. **What did you observe?** Get the raw symptom — error message, wrong output, crash, hang, slow timing, unexpected physical behaviour. Be concrete. "It's broken" is not a symptom; "the robot leg extends past 90° when given a 45° command" is. "The IMU reports heading drift of 15° over 30 seconds" is.

2. **What did you expect?** Get the intended behaviour in the same concrete terms. "The leg should stop at the commanded angle."

3. **What triggers it?** Steps, inputs, configuration, timing, environment. Even a 1-in-100 flake is useful.

4. **When was it last working?** Pin the regression window if known. Otherwise note as a data gap.

5. **What's the blast radius?** Safety hazard? Data loss? Cosmetic?

### 0.2 Read the design docs

Read `CONTEXT.md` and any ADRs in `docs/adr/` relevant to the symptom's area. For each relevant ADR, ask:

- "ADR-NNNN defines X behaviour — is this bug a violation of that decision?"
- "Does the ADR specify expected ranges, timing constraints, or event ordering that this symptom violates?"

If the user describes intended behaviour that **contradicts** an existing ADR, flag it immediately: "ADR-0004 says the leg controller should clip commands to 0–90°, but you're saying it should reject commands outside that range — which is the intended behaviour?"

**If the user and the ADRs agree but the code disagrees, you have a clear bug — the code is violating the design.**

**If the user, the ADRs, and the code all disagree with each other, you have a design drift problem — pause and resolve which is authoritative.**

### 0.3 Read the code to achieve system understanding

Now read the code in the affected area. Do NOT try to find the bug yet — just build a mental model. The goal is to understand:

- **The data flow:** Where does the relevant data originate? What transforms does it pass through? Where is it consumed to produce the output/behaviour that's misbehaving?
- **The module structure:** What modules/systems are involved? What are their interfaces and seams? Use the architecture vocabulary: modules, interfaces, seams, adapters.
- **The control flow:** What triggers the behaviour? Is it event-driven, periodic, request-response? What are the key decision points?
- **The coupling points:** Where does the code cross a seam into external systems (sensors, actuators, other tasks, hardware registers)?

Read broadly enough to trace the full path from trigger to symptom. Use `grep` to find the relevant entry points, then trace forward.

**Confirm your understanding with the user:**

> "Here's my understanding of the data flow: the leg angle command originates in `motion_planner.rs` → passes through `leg_controller.rs` which applies PID → reaches `motor_driver.rs` which converts to PWM. The symptom is at the motor output. I'll now trace this path looking for where the 45° command becomes 90°. Does this match your mental model of the system?"

If the user corrects your understanding, adjust and re-confirm. Do not proceed until system understanding is shared.

### 0.4 Write the gap statement

Before proceeding, capture the alignment:

> **Observed:** `<concrete symptom>`  
> **Expected:** `<concrete intended behaviour per ADRs / user>`  
> **Gap:** `<one-line summary of the delta>`

Show it to the user for confirmation. Do not proceed until agreed.

---

## Phase 1 — Static code analysis: trace the divergence

Now, with system understanding and an aligned target, trace the code to find where it diverges from intended behaviour. This is the **first diagnosis pass** — many bugs are visible in the code once you know the data flow and the expected outcome.

### 1.1 Trace the data flow against the expected behaviour

Starting from the trigger, walk the code path step by step. At each step, ask:

- **What value should be here?** (per the aligned expected behaviour from Phase 0)
- **What value will actually be produced?** (per the code)
- **Do they match?**

Look for:

- **Off-by-one errors** — loops, bounds, indices
- **Wrong units or coordinate frames** — degrees vs radians, global vs local, milliseconds vs microseconds
- **Missing transformations** — a calibration step, a filter, a bounds check that should exist
- **Inverted logic** — `>` instead of `>=`, `&&` instead of `||`
- **Stale or wrong constants** — hardcoded values that don't match the ADR or spec
- **Ordering violations** — operation A happens before operation B but should happen after
- **Missing error handling** — a `Result` that's silently discarded, a sensor read that can fail
- **Type coercion issues** — silent truncation, integer division where float is needed, sign mismatch
- **Lock or synchronization issues** — shared state accessed without the documented lock order
- **ADR violations** — code that directly contradicts a documented architectural decision

### 1.2 If a root cause is found → grill fix avenues

If the trace reveals a clear divergence point — a specific line or block where the code produces something different from what's expected — you have identified a **candidate root cause**.

**Do NOT apply the fix yet.** First, present the finding and grill fix avenues:

> "I traced the angle command from `motion_planner.rs:42` through the PID controller. The divergence is at `leg_controller.rs:156` — the PID output is `angle_cmd + pid_output` but it should be `pid_output` (the command is already in the setpoint, adding it again doubles the output for positive commands). This explains why a 45° command becomes 90°.
>
> Fix avenue A: Remove the `angle_cmd +` from line 156. This is a one-line change, but it assumes the PID setpoint is set correctly upstream — I need to verify.
>
> Fix avenue B: Set the PID setpoint to 0 and keep the feedforward addition. More explicit about the separation of feedforward and feedback paths, but touches more code.
>
> Which avenue do you want to explore? Or do you see a different fix?"

**Grill each avenue**, one at a time, using the grilling discipline:

- Walk down the implications of each avenue: what else changes? What tests would need updating? Does it create a regression risk elsewhere?
- If an avenue contradicts an ADR, flag it: "Avenue A means the PID controller no longer sees the commanded angle — but ADR-0007 says the controller must monitor the command for safety limits. That suggests Avenue B."
- If an avenue introduces a new seam or abstraction, note it for post-mortem.

**Execute the chosen fix** — apply it, verify it addresses the gap, then jump to Phase 5 (regression test + cleanup + post-mortem). Skip the dynamic debugging phases — the bug was found through static analysis.

### 1.3 If the avenue proves non-viable during execution

If you start executing a fix avenue and discover it doesn't work (e.g., the underlying assumption was wrong, it causes a regression, it doesn't fix the symptom), report back:

> "Avenue A doesn't fix it — removing the feedforward addition makes the leg not move at all. The PID isn't configured as a full controller; it's only providing damping on top of the feedforward. Let me go back to the code trace and re-check my assumptions. Or do you want to try Avenue B?"

If all avenues from the initial root cause are exhausted without fixing the symptom, the root cause may be wrong. Fall through to Phase 2 — the code trace didn't find the true cause, so dynamic debugging is needed.

### 1.4 If no root cause is found via static analysis

If you complete the code trace and the code appears correct — every step produces what it should produce, the logic matches the ADRs, the units and coordinate frames are consistent — then the bug is likely external to the code. State this explicitly:

> "I've traced the full path from command to motor output. The code is correct by design — every transformation is as specified, every bounds check is in place. The 90° output should be impossible for a 45° input. This suggests the bug's source is external: sensor data feeding the setpoint, a timing issue, a hardware fault, or an event misfire. I'll switch to dynamic debugging."

Proceed to Phase 2.

---

## Phase 2 — Dynamic debugging: build a feedback loop

The bug wasn't found in the code — it manifests at runtime. Build a feedback loop to observe it.

**This is the skill.** If you have a **tight** pass/fail signal for the bug, you will find the cause. If you don't, no amount of staring will save you.

### Ways to construct one — try them in roughly this order

1. **Failing test** at whatever seam reaches the bug — unit, integration, e2e.
2. **CLI invocation** with a fixture input, diffing stdout against a known-good snapshot.
3. **Replay a captured trace.** Save a real payload / event log to disk; replay it through the code path in isolation.
4. **Throwaway harness.** Minimal subset of the system that exercises the bug code path.
5. **Property / fuzz loop.** 1000 random inputs, look for the failure mode.
6. **Differential loop.** Old-version vs new-version, diff outputs.
7. **HITL log-driven loop.** See Phase 2b. For embedded/physical bugs where the agent can't run the system unattended.

### Tighten the loop

Can you make it faster? Sharper? More deterministic? A 2-second deterministic loop is a superpower; a 30-second flaky one is barely useful.

### Non-deterministic bugs

Goal: higher reproduction rate. Loop 100×, parallelise, add stress, narrow timing windows. 50%-flake is debuggable; 1% is not.

### Completion criterion

Phase 2 is done when you can name **one command or procedure** that you have **already run** (paste invocation + output), that is:

- [ ] **Red-capable** — drives the actual bug path and asserts the user's exact symptom from Phase 0.4
- [ ] **Deterministic** (or high repro rate)
- [ ] **Fast** — seconds, not minutes
- [ ] **Agent-runnable** (for HITL: agent drives procedure, human is the instrument)

---

## Phase 2b — HITL log-driven loop (for data/signal/event/hardware bugs)

When the bug source is outside the code (sensor data, physical events, hardware, timing, environment), the feedback loop involves a human running the system and reporting back.

### 2b.1 Instrument the data flow

Add targeted `defmt!` (or equivalent) instrumentation at the **boundaries** of the suspected data flow:

1. **Source** — right after sensor read, ADC conversion, event capture. Log the raw value.
2. **Transform** — after calibration, filtering, coordinate transform. Log the intermediate.
3. **Decision** — just before the code acts on the value (threshold check, control output).
4. **Effect** — the actuator command, state transition, or output.

Tag every log with a session-unique prefix, e.g. `[HITL-a4f2]`. Cleanup is a single grep.

Example for an IMU heading drift bug:

```rust
defmt::debug!("[HITL-a4f2] raw_gyro_z={} dps", raw_gyro_z);
defmt::debug!("[HITL-a4f2] filtered_gyro_z={} dps", filtered_gyro_z);
defmt::debug!("[HITL-a4f2] heading_deg={}", heading_deg);
defmt::debug!("[HITL-a4f2] motor_cmd_yaw={}", motor_cmd_yaw);
```

**Keep instrumentation minimal** — instrument only the data path directly related to the symptom.

### 2b.2 Define the HITL procedure

Give the user a concrete, reproducible, bounded procedure:

> **HITL procedure #1 — IMU drift observation**
>
> 1. Flash firmware with `[HITL-a4f2]` instrumentation.
> 2. Place robot on a level surface, stationary, powered on.
> 3. Connect debug probe and log: `probe-rs run --chip RP2350 | tee hitl-1.log`
> 4. Let it run for 60 seconds. Do not touch the robot.
> 5. Stop logging. Send me `hitl-1.log`.

### 2b.3 Analyze the logs

When the user returns logs, analyze them against the design tree:

1. **Does the data flow match the ADR's expected path?**
2. **Are values within expected ranges?** Flag any value outside spec from ADRs.
3. **Is the data flow complete?** Any missing transforms, skipped filters, zero-where-should-be-nonzero?
4. **Where is the divergence point?** Find the exact log line where values go from correct to incorrect.

Report findings to the user:

> "Log analysis: raw gyro reads 0.0 dps (correct for stationary), filtered gyro reads 0.0 dps (correct), but heading_deg increases at 0.5°/s. The divergence is between filtered_gyro_z and heading_deg. The integration step is accumulating an offset."

### 2b.4 Iterate — narrow the instrumentation

Add more targeted instrumentation around the suspect area. Remove instrumentation from paths confirmed correct. Each iteration narrows the suspect region and adds a concrete prediction. Repeat until root cause is isolated.

### 2b.5 Stop conditions for HITL

- **Root cause isolated in code** → proceed to Phase 3 (hypothesise), then Phase 5 (fix)
- **Root cause is external** (sensor fault, mechanical, EMI, power) → document with log evidence. Ask: "Should we add a code-level defense (sanity check, timeout, validation gate)? Should the ADR specify one?"
- **Cannot isolate after 5 iterations** → reassess. Consider higher-frequency logging, oscilloscope/protocol analyzer, reclassifying the source.

---

## Phase 3 — Hypothesise

Generate **3–5 ranked hypotheses** before testing any of them. Each hypothesis must be **falsifiable**:

> "If `<X>` is the cause, then `<changing Y>` will make the bug disappear / `<changing Z>` will make it worse."

If you cannot state the prediction, discard or sharpen the hypothesis.

### Grill the user on the ranking

Show the ranked list. State your reasoning. **Ask one question per hypothesis, waiting for feedback.** The user's domain knowledge may re-rank instantly.

If the user is AFK, proceed with your ranking — note the lack of domain confirmation.

---

## Phase 4 — Instrument

Each probe maps to a specific prediction from Phase 3. **Change one variable at a time.**

Tool preference: debugger/REPL first, targeted logs second. Never "log everything and grep."

Tag every debug log with a unique prefix, e.g. `[DEBUG-a4f2]`.

As probes narrow the cause, surface findings to the user: "`[DEBUG-a4f2]` confirms the divergence is inside the controller module. Align with your understanding?"

---

## Phase 5 — Fix + regression test + cleanup + post-mortem

### 5.1 Regression test

Write the regression test **before the fix** — but only if there is a **correct seam** for it (exercises the real bug pattern). If no correct seam exists, note it — the architecture is preventing the bug from being locked down (flag for post-mortem).

For HITL bugs: the regression test injects the bad data pattern discovered in Phase 2b. No hardware needed.

### 5.2 Apply the fix

1. Turn the minimised repro into a failing test at the correct seam.
2. Watch it fail.
3. Apply the fix.
4. Watch it pass.
5. Re-run the Phase 2 feedback loop against the original scenario.

### 5.3 Cleanup

- [ ] Original repro no longer reproduces
- [ ] Regression test passes (or absence of seam is documented)
- [ ] All `[DEBUG-...]` and `[HITL-...]` instrumentation removed (`grep` the prefix)
- [ ] Throwaway prototypes deleted
- [ ] Correct hypothesis stated in commit/PR message

### 5.4 Grill with docs — the post-mortem

Walk through these questions with the user, one at a time. Use the `domain-modeling` discipline for docs.

1. **What would have prevented this bug?** If architectural, hand off to `/improve-codebase-architecture`. If a missing constraint, offer an ADR.

2. **Did this bug reveal a missing domain concept?** If terminology was inconsistent, sharpen it in CONTEXT.md.

3. **Did this bug contradict an ADR?** If the code violated an ADR, note it. If the ADR was wrong, offer to supersede it.

4. **Did we make an architectural trade-off?** If the fix introduces a new seam or changes coupling, consider an ADR.

5. **Is there a pattern?** Nth bug in same module or same class? Flag it.

6. **For hardware/external root causes:** Did we add a code-level defense? Should it be encoded as a defensive-design ADR?

**End with:** "The gap from Phase 0.4 is now closed. Observed = expected. Anything else you want to investigate?"

---

## Reference — when to use this vs other skills

| Situation | Use |
|---|---|
| "X is broken, investigate" | `diagnosing-bugs` (this skill) |
| "The sensor data looks wrong but code seems fine" | `diagnosing-bugs` → Phase 2b HITL loop |
| "It works sometimes but not others" | `diagnosing-bugs` → likely event/timing, Phase 2b |
| "Review my code for issues" | `code-review` |
| "Is this codebase well-designed?" | `improve-codebase-architecture` |
| "Stress-test my plan before I build" | `grill-me` or `grill-with-docs` |

This skill hands off to `domain-modeling` (for CONTEXT.md/ADR updates) and `improve-codebase-architecture` (for post-mortem architectural findings). It does not replace them.

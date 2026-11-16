# Milestone progress

State file for the `/milestone-step` loop (`.claude/skills/milestone-step/SKILL.md`). The loop reads and updates this file on every run. Edit the `Answer:` line to unblock it.

- **Milestone:** M1 (headless core, STEP to URDF + MJCF)
- **Branch:** `m1-core`
- **Status:** READY

## Pending question

(none)

## Answered questions

- 2026-11-16: Uncommitted changes are on `m1-core` (the re-exported tibia frame was committed separately): `CLAUDE.md`, `doc/milestones.md`, `doc/rigid-forge-software-stack.md`, `doc/milestone-progress.md` and `.claude/skills/milestone-step/`. The loop will not start task 1 on top of uncommitted work it did not make. May it commit these as one "planning docs and fixtures" commit on `m1-core` and then start task 1?
    Recommended: yes.
    Answer: yes

## M1 status

Spikes (findings in design doc section 6, "Spike findings"):

| Item | Status |
|---|---|
| Spike: STEP units and XCAF assembly | DONE (2026-11-15) |
| Spike: inertia accuracy | DONE (2026-11-15) |
| Spike: URDF vs MJCF semantics | DONE (2026-11-15) |
| Spike: mirror correctness | DONE (2026-11-15) |
| Spike: leg pair placement and horn seating | DONE (2026-11-16) |
| Spike: real 3MF file (part of task 8) | TODO |

Tasks (numbers and dependencies as in `doc/milestones.md`):

| # | Task | Status |
|---|---|---|
| 1 | Scaffold (pyproject, uv lock, .gitignore, markers, ruff + pyright) | TODO |
| 2 | Model (pydantic, UUIDs, extensions) | TODO |
| 3 | Validation | TODO |
| 4 | Analytic inertia + checks | TODO |
| 5 | Synthetic fixture generator | TODO |
| 6 | STEP importer | TODO |
| 7 | B-Rep mass properties | TODO |
| 8 | Mesh importer (STL, OBJ, DAE, 3MF) | TODO |
| 9 | Persistence | TODO |
| 10 | URDF exporter | TODO |
| 11 | MJCF exporter | TODO |
| 12 | CLI | TODO |
| 13 | Round-trip, determinism and layering tests | TODO |
| 14 | Design doc update from spikes | TODO |
| 15 | Instancing and mirror import | TODO |
| 16 | Servo assembly | TODO |
| 17 | Measured-mass calibration | TODO |
| 18 | Servo inertia override | TODO |
| 19 | Leg assembly script | TODO (needs: joint limits and CAD pose angles, measured link masses, XL330 inertia and CoM source) |

## GUI checks for the user

(none yet)

## Run log

- 2026-11-16: loop set up; waiting for the commit question above.
- 2026-11-16: answer "yes" applied; planning docs, CLAUDE.md and the skill committed on `m1-core`.

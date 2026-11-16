---
name: milestone-step
description: Do the next step of the RigidForge milestone plan (doc/milestones.md), tracked in doc/milestone-progress.md. Resumable after interruption or reboot; stops and records a question whenever a decision belongs to the user. Run manually as /milestone-step or from the 2-hourly scheduled task.
---

# Milestone step

Work through the RigidForge milestones one task at a time. Each invocation does **at most one task** (or one spike), then stops. All state lives in files, so any fresh session can pick up where the last one stopped.

Repository: `/home/stephen/Workspace/rigid-forge`. Work there for every command. `uv` is in `~/.local/bin`; add it to `PATH` if needed.

## Files

- `CLAUDE.md`: project rules, including the "Milestone workflow" section. Read it first and follow it.
- `doc/milestones.md`: the plan (tasks, dependencies, open items).
- `doc/rigid-forge-software-stack.md`: the authoritative design doc.
- `doc/milestone-progress.md`: the state file. It holds the current milestone and branch, the status of every task, the run log, and any pending question with its answer.
- Lock file: `/home/stephen/.claude/rigid-forge-milestone.lock` (outside the repo).

## Procedure

1. **Lock.** If the lock file exists and is less than 3 hours old, another run is active: stop and do nothing. Otherwise (missing, or stale after a crash or reboot) write the current ISO timestamp and session start to it. Remove it at the end of every run, including runs that stop early.

2. **Read state.** Read `CLAUDE.md`, `doc/milestone-progress.md` and the current milestone's section of `doc/milestones.md`.

3. **Pending question.** If the progress file's `Status` is `BLOCKED`:
   - If the `Answer:` line under "Pending question" is still empty, do nothing else. Log "still waiting for an answer" in the run log, release the lock, and stop. Do not re-ask in other words or guess an answer.
   - If it has an answer, apply it (record any decision in the design doc or milestones doc, as CLAUDE.md requires), move the question and answer to "Answered questions", set `Status: READY`, and continue.

4. **Check the working tree.** Run `git status` and `git branch --show-current`.
   - Be on the milestone branch named in the progress file (create it from `main` if it does not exist). Never commit to `main`.
   - If a task is marked `IN PROGRESS` and there are uncommitted changes, the previous run was interrupted (for example by a reboot). Review the diff and finish that task rather than starting a new one.
   - If there are uncommitted changes that no `IN PROGRESS` task explains, do not touch them: ask about them as a question (step 7).

5. **Pick the next task.** Choose the first task in the current milestone that is not `DONE` and whose dependencies (`->` in `doc/milestones.md`) are all `DONE`. Spikes come before tasks. If the milestone has open "Open before ..." items that the chosen task depends on, ask about them (step 7) instead of starting. Mark the task `IN PROGRESS` in the progress file before writing code.

6. **Do the task.** Follow CLAUDE.md:
   - Write the code and its tests.
   - Run `uv run ruff check`, `uv run ruff format --check`, `uv run pyright` and `uv run pytest` (once task 1 has set them up). All must pass.
   - Verify `(verify)` items against primary sources, not memory.
   - Anything in `ui/` or `viewport/` is untested in the GUI: write a manual checklist into the progress file for the user, and mark the task `DONE (needs GUI check)`.
   - Commit once, on the milestone branch, with a message naming the task, ending with the attribution line your session instructions give. Never push, merge, tag or open a PR.
   - Mark the task `DONE` with the commit hash, and add a one-line entry to the run log.

7. **Asking a question (pausing).** Whenever a decision is the user's (anything CLAUDE.md says to ask about, an unresolved open item, a failing check you cannot fix within the task, unexpected uncommitted changes, or a milestone's done definition being met):
   - Set `Status: BLOCKED`.
   - Write one clear question under "Pending question" with the context needed to answer it, your recommended answer, and an empty `Answer:` line.
   - Leave any partial work uncommitted and the task `IN PROGRESS`, or revert it if it is broken, and say which in the run log.
   - Send a push notification (PushNotification tool, if available) saying the loop is paused and what it is asking.
   - Release the lock and stop.

8. **Milestone close-out.** When every task in the milestone is `DONE`, check its "Done =" definition end to end and record the result. Then ask (step 7) whether to merge the branch and start the next milestone. Do not merge or tag yourself.

9. **Finish.** Release the lock. End with a short summary: what was done, the commit hash, and the next task (or the pending question).

## Answering

The user answers by filling in the `Answer:` line in `doc/milestone-progress.md`, or by telling a running session, which must then write the answer to that line itself. Answers given only in chat are lost when the session ends.

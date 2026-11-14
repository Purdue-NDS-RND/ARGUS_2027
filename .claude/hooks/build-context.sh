#!/usr/bin/env bash
# Re-injects the build session's state after the context is compacted or the
# session is resumed. Wired up only by docs/build/run-settings.json, so it never
# runs in ordinary sessions. Whatever it prints is added to Claude's context.

cd "${CLAUDE_PROJECT_DIR:-.}" 2>/dev/null || exit 0

echo "ARGUS /build session: the context was just compacted or resumed."
echo "Before anything else, re-read docs/build/SESSION.md in full, then carry on with the loop at the top of the /build skill."
echo "If a row is DOING, check what is already committed for it: git log --oneline --grep '^<ID>:'"
echo
python3 tools/build_ledger.py summary 2>/dev/null || echo "(ledger summary unavailable)"
echo
echo "Recent commits:"
git log --oneline -5 2>/dev/null || true
echo
echo "Newest build-log lines (docs/build/PROGRESS.md):"
grep -m 5 -E '^- (20[0-9]{2}-|STOP)' docs/build/PROGRESS.md 2>/dev/null || echo "(none yet)"
exit 0

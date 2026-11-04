#!/usr/bin/env bash
# SessionStart hook: point git at the repo's hooks in .githooks, so agent sessions
# get pre-commit and commit-msg with no setup step. Prints nothing when they are
# already on. Fails open: on any problem it prints why (Claude sees the line) and exits 0.

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd) || {
    echo "SessionStart hook: could not find the repo root, git hooks not set up"
    exit 0
}
if [[ ! -d $root/.githooks ]]; then
    echo "SessionStart hook: $root/.githooks is missing, git hooks not set up"
    exit 0
fi
if [[ $(git -C "$root" config --get core.hooksPath 2>/dev/null) == .githooks ]]; then
    exit 0
fi
if out=$(git -C "$root" config core.hooksPath .githooks 2>&1); then
    echo "SessionStart hook: set git core.hooksPath to .githooks, so the pre-commit and commit-msg hooks are on"
else
    echo "SessionStart hook: git config core.hooksPath .githooks failed, so the git hooks are off: $out"
fi
exit 0

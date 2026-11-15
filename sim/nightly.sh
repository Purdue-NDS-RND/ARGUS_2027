#!/usr/bin/env bash
# Tier B: the nightly sim run. Meant for a club machine that never sleeps,
# started by cron or a systemd timer; nobody watches it.
#
#   sim/nightly.sh                 fly the nightly suite on main, post the table
#   sim/nightly.sh --suite smoke   another suite
#   sim/nightly.sh --dry-run       say what it would do
#
# Each run:
#   1. fetches origin and makes a fresh worktree of main under
#      sim/.state/nightly/worktree, so the night's result is main's, whatever
#      this checkout has in it,
#   2. runs that worktree's own sim/run_scenarios.sh <suite> (the nightly
#      suite flies every scenario, three seeds, flags off and on),
#   3. posts the suite's table as a comment on the GitHub issue numbered
#      ARGUS_NIGHTLY_ISSUE, using the `gh` CLI. With no issue set it prints
#      the report instead.
# Exit code 0 only if every flight was OK and the report was delivered.
#
# Not a self-hosted Actions runner on purpose: the repo is public, and a
# runner would execute any pull request's code on the club machine.
#
# `gh` reads its token from its own login or from the GH_TOKEN variable. Give
# it a fine-grained token that can only write issues on this repo. This
# script never prints or stores it.
set -Eeuo pipefail

SIM_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=${NIGHTLY_REPO:-$(cd -- "$SIM_DIR/.." && pwd)}
REF=${NIGHTLY_REF:-origin/main}
WORK=${NIGHTLY_DIR:-$SIM_DIR/.state/nightly}
ISSUE=${ARGUS_NIGHTLY_ISSUE:-}
suite=nightly
dry_run=0

while (($#)); do
    case $1 in
        --suite) suite=${2:?--suite needs a name}; shift ;;
        --dry-run) dry_run=1 ;;
        --help|-h) sed -n '2,25p' "$0"; exit 0 ;;
        *) echo "ERROR: unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

if ((dry_run)); then
    echo "would fetch origin in $REPO and make a fresh worktree of $REF at $WORK/worktree"
    echo "would run there: sim/run_scenarios.sh $suite"
    if [[ -n $ISSUE ]]; then
        echo "would post the report as a comment on issue #$ISSUE"
    else
        echo "would print the report (ARGUS_NIGHTLY_ISSUE is not set)"
    fi
    exit 0
fi

mkdir -p "$WORK"
tree="$WORK/worktree"
report="$WORK/report.md"

# A fresh worktree every night. The old one holds last night's sim state and
# build; it is this script's own scratch and nothing else lives in it.
if [[ $REF == origin/* ]]; then
    git -C "$REPO" fetch --quiet origin
fi
if [[ -e $tree ]]; then
    git -C "$REPO" worktree remove --force "$tree" 2>/dev/null || rm -rf -- "$tree"
fi
git -C "$REPO" worktree prune
git -C "$REPO" worktree add --quiet --detach "$tree" "$REF"
commit=$(git -C "$tree" rev-parse --short HEAD)
subject=$(git -C "$tree" log -1 --format=%s)

status=0
"$tree/sim/run_scenarios.sh" "$suite" >"$WORK/run.log" 2>&1 || status=$?

summary=$(sed -n 's/^Summary: //p' "$WORK/run.log" | tail -n 1)
{
    echo "## Sim suite \`$suite\` on main at \`$commit\`, $(date -u +%Y-%m-%d)"
    echo
    echo "Commit: $subject"
    echo
    if ((status == 0)); then
        echo "**Result: every flight OK.**"
    else
        echo "**Result: NOT every flight was OK** (the runner exited $status)."
    fi
    echo
    if [[ -n $summary && -f $tree/$summary ]]; then
        sed '1{/^# /d}' "$tree/$summary"
    else
        echo "The runner left no summary. Its last lines:"
        echo
        echo '```'
        tail -n 20 "$WORK/run.log"
        echo '```'
    fi
} >"$report"

if [[ -z $ISSUE ]]; then
    cat "$report"
    echo
    echo "Not posted: set ARGUS_NIGHTLY_ISSUE to the pinned issue's number to post this as a comment."
    exit "$status"
fi
if ! gh issue comment "$ISSUE" --body-file "$report" >/dev/null; then
    echo "ERROR: could not post the report to issue #$ISSUE (is gh logged in, with a token that can write issues?)." >&2
    echo "The report is in $report." >&2
    exit 1
fi
echo "Posted the $suite report for $commit to issue #$ISSUE."
exit "$status"

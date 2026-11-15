# When SITL's stored parameters must be wiped. Sourced by start.sh.
# shellcheck shell=bash
#
# SITL keeps parameters and the geofence in its storage directory between
# starts. A site run uploads a fence, so:
#   - a site run always starts from wiped storage ("scenario runs always wipe
#     SITL's stored parameters"), and
#   - the first plain run after a site run wipes too, so a site's fence never
#     follows the vehicle to the 2026 world, where it could not arm.
# A plain run after a plain run does not wipe: that is the 2026 behaviour.
# <dir>/last_site remembers which kind of run used the storage last.

# sitl_wipe_needed <sitl dir> <site file, or empty> <WIPE_PARAMS>: prints 1 or 0.
sitl_wipe_needed() {
    local dir=$1 site=$2 last
    case "${3:-0}" in
        1|true|yes|on) echo 1; return 0 ;;
    esac
    last=$(cat "$dir/last_site" 2>/dev/null || echo none)
    if [[ -n $site || $last != none ]]; then
        echo 1
    else
        echo 0
    fi
}

# sitl_mark_storage <sitl dir> <site file, or empty>: record who uses the storage.
# start.sh marks a site run before SITL starts (the storage is the site's from
# then on) and a plain run only once SITL is up, so an interrupted start can
# never clear the mark before the wipe has happened.
sitl_mark_storage() {
    echo "${2:-none}" >"$1/last_site"
}

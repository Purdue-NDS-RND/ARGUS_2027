#/!/usr/bin/env python3
import subprocess
import logging

_logger = logging.getLogger(__name__)


def _build_box_sdf(model_name: str, size, rgba) -> str:
    sx, sy, sz = size
    r, g, b, a = rgba
    return (
        "<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        "<static>true</static>"
        "<link name='link'>"
        "<collision name='collision'>"
        "<geometry>"
        f"<box><size>{sx} {sy} {sz}</size></box>"
        "</geometry>"
        "</collision>"
        "<visual name='visual'>"
        "<geometry>"
        f"<box><size>{sx} {sy} {sz}</size></box>"
        "</geometry>"
        "<material>"
        "<ambient>"
        f"{r} {g} {b} {a}"
        "</ambient>"
        "<diffuse>"
        f"{r} {g} {b} {a}"
        "</diffuse>"
        "</material>"
        "</visual>"
        "</link>"
        "</model>"
        "</sdf>"
    )


def _build_digit_sdf(digit: int, cx: float, cy: float, size: float, z: float) -> str:
    """Build SDF elements for a 7-segment style digit.
    
    Segments layout:
       AAA
      F   B
       GGG
      E   C
       DDD
    
    Args:
        digit: 0-9
        cx, cy: center position
        size: overall height of digit
        z: z position
    """
    # Segment dimensions
    seg_len = size * 0.4   # Horizontal segment length
    seg_w = size * 0.12    # Segment width/thickness
    v_len = size * 0.35    # Vertical segment length
    
    # Positions relative to center
    top_y = cy + size * 0.35
    mid_y = cy
    bot_y = cy - size * 0.35
    left_x = cx - seg_len * 0.4
    right_x = cx + seg_len * 0.4
    
    # Which segments are on for each digit (A,B,C,D,E,F,G)
    segments = {
        0: [1,1,1,1,1,1,0],
        1: [0,1,1,0,0,0,0],
        2: [1,1,0,1,1,0,1],
        3: [1,1,1,1,0,0,1],
        4: [0,1,1,0,0,1,1],
        5: [1,0,1,1,0,1,1],
        6: [1,0,1,1,1,1,1],
        7: [1,1,1,0,0,0,0],
        8: [1,1,1,1,1,1,1],
        9: [1,1,1,1,0,1,1],
    }
    
    segs = segments.get(digit, segments[0])
    result = ""
    mat = "<material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>"
    
    # A - top horizontal
    if segs[0]:
        result += (
            f"<link name='seg_a'><pose>{cx:.4f} {top_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # B - top right vertical
    if segs[1]:
        result += (
            f"<link name='seg_b'><pose>{right_x:.4f} {(top_y+mid_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # C - bottom right vertical
    if segs[2]:
        result += (
            f"<link name='seg_c'><pose>{right_x:.4f} {(mid_y+bot_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # D - bottom horizontal
    if segs[3]:
        result += (
            f"<link name='seg_d'><pose>{cx:.4f} {bot_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # E - bottom left vertical
    if segs[4]:
        result += (
            f"<link name='seg_e'><pose>{left_x:.4f} {(mid_y+bot_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # F - top left vertical
    if segs[5]:
        result += (
            f"<link name='seg_f'><pose>{left_x:.4f} {(top_y+mid_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # G - middle horizontal
    if segs[6]:
        result += (
            f"<link name='seg_g'><pose>{cx:.4f} {mid_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    
    return result


def build_bw_target_sdf(model_name: str, size: float = 0.61, digit: int = 1) -> str:
    """Build SDF for black-white GCP target with X pattern and number.
    
    Creates a 24" (0.61m) square target with:
    - White base
    - Black triangles on top and bottom (created with strips)
    - White triangles on left and right  
    - A digit (0-9) with underline in bottom-right quadrant
    
    Args:
        model_name: Name for the model
        size: Side length in meters (default 0.61m = 24")
        digit: Number to display (0-9)
    """
    half = size / 2.0
    thickness = 0.01  # 1cm thick plate
    layer_z = thickness / 2 + 0.001  # Z position for pattern layers
    
    # Build triangles using horizontal strips
    # Each strip gets narrower as we approach the center
    num_strips = 10
    strip_height = half / num_strips
    
    # Generate strips for top triangle (black)
    top_strips = ""
    for i in range(num_strips):
        # y position from center toward top
        y_pos = (i + 0.5) * strip_height
        # Width decreases linearly from full width at top to 0 at center
        # At y=half (top edge), width = size
        # At y=0 (center), width = 0
        progress = 1.0 - (y_pos / half)  # 1 at center, 0 at top
        strip_width = size * (1.0 - progress * 0.95)  # Don't go to 0, leave small tip
        
        top_strips += (
            f"<link name='top_strip_{i}'>"
            f"<pose>0 {y_pos:.4f} {layer_z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><box><size>{strip_width:.4f} {strip_height:.4f} 0.002</size></box></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>"
            f"</visual>"
            f"</link>"
        )
    
    # Generate strips for bottom triangle (black)
    bottom_strips = ""
    for i in range(num_strips):
        # y position from center toward bottom (negative y)
        y_pos = -((i + 0.5) * strip_height)
        progress = 1.0 - (abs(y_pos) / half)
        strip_width = size * (1.0 - progress * 0.95)
        
        bottom_strips += (
            f"<link name='bottom_strip_{i}'>"
            f"<pose>0 {y_pos:.4f} {layer_z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><box><size>{strip_width:.4f} {strip_height:.4f} 0.002</size></box></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>"
            f"</visual>"
            f"</link>"
        )
    
    # Number in bottom-right white area (positioned in right triangle)
    num_x = half * 0.65  # More to the right
    num_y = -half * 0.05
    num_size = size * 0.42  # Bigger digit
    
    # Build digit using 7-segment style boxes
    number_elements = _build_digit_sdf(digit, num_x, num_y, num_size, layer_z + 0.001)
    
    # Add underline below the digit (fixed size, not tied to num_size)
    underline_y = num_y - num_size * 0.75
    underline_w = size * 0.18  # Fixed width
    underline_h = size * 0.025  # Fixed height
    number_elements += (
        f"<link name='underline'>"
        f"<pose>{num_x:.4f} {underline_y:.4f} {layer_z + 0.001:.4f} 0 0 0</pose>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{underline_w:.4f} {underline_h:.4f} 0.002</size></box></geometry>"
        f"<material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>"
        f"</visual>"
        f"</link>"
    )
    
    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        
        # Base white plate
        f"<link name='base'>"
        f"<pose>0 0 0 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size} {size} {thickness}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size} {size} {thickness}</size></box></geometry>"
        f"<material>"
        f"<ambient>0.95 0.95 0.95 1</ambient>"
        f"<diffuse>0.95 0.95 0.95 1</diffuse>"
        f"</material>"
        f"</visual>"
        f"</link>"
        
        # Top triangle strips
        f"{top_strips}"
        
        # Bottom triangle strips  
        f"{bottom_strips}"
        
        # Number and underline
        f"{number_elements}"
        
        f"</model>"
        f"</sdf>"
    )

def build_red_white_target_sdf(model_name: str, size: float = 5.0, num_rings: int = 8) -> str:
    """Build SDF for a red-white bullseye target for Package Drop/Delivery missions.

    Creates a circular target with alternating red and white concentric rings
    and a red center dot, matching competition spec (~5m diameter).

    The pattern from outside inward is:
        white base → red → white → red → white → red (center)

    Args:
        model_name: Name for the Gazebo model.
        size:       Diameter in meters (default 5.0m per competition rules).
        num_rings:  Number of visible ring layers over the white base (default 10,
                    giving 5 red + 5 white alternating bands + red center dot).
    """
    radius = size / 2.0
    thickness = 0.02        # 2 cm thick base plate
    layer_step = 0.002      # 2 mm z-separation per layer to prevent z-fighting

    RED   = "0.85 0.05 0.05 1"
    WHITE = "0.95 0.95 0.95 1"

    links = ""

    # --- White square tarp base (full size x size) ---
    base_z = thickness / 2.0
    links += (
        f"<link name='base'>"
        f"<pose>0 0 {base_z:.4f} 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"<material><ambient>{WHITE}</ambient><diffuse>{WHITE}</diffuse></material>"
        f"</visual>"
        f"</link>"
    )

    # --- Concentric ring overlays: red outermost, alternating inward ---
    # Divide radius evenly among (num_rings + 1) bands so the innermost band
    # is also one unit wide (it becomes the center dot below).
    band_width = radius / (num_rings + 1)

    for i in range(num_rings):
        # Ring index 0 = outermost red ring, increases inward
        ring_radius = radius - i * band_width
        color = RED if (i % 2 == 0) else WHITE
        z = thickness + (i + 1) * layer_step
        links += (
            f"<link name='ring_{i}'>"
            f"<pose>0 0 {z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><cylinder><radius>{ring_radius:.4f}</radius><length>0.001</length></cylinder></geometry>"
            f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"
            f"</visual>"
            f"</link>"
        )

    # --- Center red dot ---
    center_radius = band_width
    center_z = thickness + (num_rings + 1) * layer_step
    links += (
        f"<link name='center'>"
        f"<pose>0 0 {center_z:.4f} 0 0 0</pose>"
        f"<visual name='visual'>"
        f"<geometry><cylinder><radius>{center_radius:.4f}</radius><length>0.001</length></cylinder></geometry>"
        f"<material><ambient>{RED}</ambient><diffuse>{RED}</diffuse></material>"
        f"</visual>"
        f"</link>"
    )

    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        f"{links}"
        f"</model>"
        f"</sdf>"
    )


def spawn_sdf(
    sdf_str: str,
    model_name: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    world_name: str = "map",
) -> bool:
    """Spawn an SDF model into a running Gazebo simulation.

    Uses the same ``gz service`` CLI approach as the TargetSpawner node.
    No ROS node or context is required; the function is safe to call from
    any thread or standalone script as long as ``gz`` is on PATH and
    Gazebo is running.

    Args:
        sdf_str:    Raw SDF XML string (e.g. returned by ``build_*_sdf`` helpers).
        model_name: Unique name to give the spawned entity.
        x:          World-frame X position in metres.
        y:          World-frame Y position in metres.
        z:          World-frame Z position in metres.
        world_name: Gazebo world name (default ``"default"``).

    Returns:
        ``True`` if Gazebo confirmed the spawn, ``False`` otherwise.
    """
    # Escape quotes for shell (matches target_spawner._spawn_via_gz_cli)
    sdf_escaped = sdf_str.replace('"', '\\"')

    req_msg = (
        f'sdf: "{sdf_escaped}" '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}} '
        f'name: "{model_name}" '
        f'allow_renaming: true'
    )

    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", req_msg
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
        if result.returncode == 0 and "true" in result.stdout.lower():
            _logger.info(f"Spawned '{model_name}' at ({x:.2f}, {y:.2f}, {z:.2f})")
            return True
        else:
            _logger.warning(f"Spawn failed: {result.stderr or result.stdout}")
            return False
    except subprocess.TimeoutExpired:
        _logger.warning("Spawn command timed out")
        return False
    except FileNotFoundError:
        _logger.error("'gz' command not found - is Gazebo installed?")
        return False

# --- 2027 sim objects -------------------------------------------------------
# Used by the scenario builder (sim/scenario/build.py), which embeds them in
# the world file. The 2026 builders above are left exactly as they flew.

_WHITE = "0.95 0.95 0.95 1"
_PLATE_BLACK = "0.05 0.05 0.05 1"
_X_GREEN = "0 0.627 0 1"          # tools/synth_targets.py XMat.color (0, 160, 0)
_BUCKET_ORANGE = "0.922 0.431 0.078 1"  # tools/synth_targets.py bucket_color (235, 110, 20)


def _material(rgba: str) -> str:
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def _flat_link(name: str, x: float, y: float, z: float, yaw: float, sx: float, sy: float, rgba: str) -> str:
    """A 2 mm thick coloured rectangle lying flat: a visual only, no collision."""
    return (
        f"<link name='{name}'>"
        f"<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{sx:.4f} {sy:.4f} 0.002</size></box></geometry>"
        f"{_material(rgba)}"
        f"</visual>"
        f"</link>"
    )


def build_marker_sdf(model_name: str, size: float = 0.6, digit=None) -> str:
    """Build SDF for a 2027 localisation marker: a black-and-white X plate with
    its digit centred on the X.

    Numbered markers are 0.6 m; the four boundary markers are 1.2 m and carry
    no digit (``digit=None``). The digit is the same seven-segment style as
    ``build_bw_target_sdf``, on a white field over the crossing point, with
    no underline. The model's +y is the top of the digit; the scenario gives
    the marker its heading through the model pose.

    Args:
        model_name: Name for the model.
        size:       Side length in metres.
        digit:      0-9, or None for a boundary marker.
    """
    if size <= 0.0:
        raise ValueError(f"marker size must be positive, got {size}")
    if digit is not None and digit not in range(10):
        raise ValueError(f"marker digit must be 0-9 or None, got {digit!r}")

    half = size / 2.0
    thickness = 0.01
    strips_z = thickness + 0.001
    field_z = thickness + 0.003
    digit_z = thickness + 0.005

    # The X: black triangles above and below the centre, white ones either side,
    # drawn as strips that narrow toward the centre.
    num_strips = 10
    strip_height = half / num_strips
    strips = ""
    for side, sign in (("top", 1.0), ("bottom", -1.0)):
        for i in range(num_strips):
            y_pos = (i + 0.5) * strip_height
            width = size * (0.05 + 0.95 * y_pos / half)
            strips += _flat_link(f"{side}_strip_{i}", 0.0, sign * y_pos, strips_z, 0.0, width, strip_height, _PLATE_BLACK)

    number = ""
    if digit is not None:
        digit_size = size * 0.5
        number = _flat_link("digit_field", 0.0, 0.0, field_z, 0.0, digit_size * 0.6, digit_size, _WHITE)
        number += _build_digit_sdf(digit, 0.0, 0.0, digit_size, digit_z)

    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        f"<link name='base'>"
        f"<pose>0 0 {thickness / 2:.4f} 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"{_material(_WHITE)}"
        f"</visual>"
        f"</link>"
        f"{strips}"
        f"{number}"
        f"</model>"
        f"</sdf>"
    )


# The AprilTag 36h11 of id 0, border included, row by row from its top left: 1 is white. It is the grid
# drone_target_cv/home_marker.py reads (a test holds the two together); written out here so that the sim's models need no OpenCV.
HOME_TAG_CELLS = ("00000000", "00010000", "00110100", "00001010", "00001100", "01011100", "01010110", "00000000")


HOME_TAG_LAYERS_M = (0.005, 0.015, 0.025)  # the board, the tag's black square and its white cells, above the ground


def build_home_tag_sdf(model_name: str, size: float = 0.8) -> str:
    """Build SDF for the home marker (REC-7b): an AprilTag 36h11 of id 0, lying flat.

    A white board one cell wider than the tag all round, the tag's black
    square on it, and the tag's white cells on that. Seen from above with
    north up, the tag's top row is to the north and its columns run east, so
    the picture is the tag and not its mirror image.

    Args:
        model_name: Name for the model.
        size:       Side of the tag across its black border, in metres.
    """
    cell = size / len(HOME_TAG_CELLS)
    white, black = "1 1 1 1", "0.02 0.02 0.02 1"
    # A centimetre between the layers: plates 2 mm apart were drawn through one another from flying height (the tag was read in one
    # frame of six in the sim's first flights), and a centimetre of height moves nothing by a pixel.
    board_z, tag_z, cells_z = HOME_TAG_LAYERS_M
    links = [_flat_link("board", 0.0, 0.0, board_z, 0.0, size + 2.0 * cell, size + 2.0 * cell, white), _flat_link("tag", 0.0, 0.0, tag_z, 0.0, size, size, black)]
    for row, line in enumerate(HOME_TAG_CELLS):
        column = 0
        while column < len(line):  # each run of white cells in a row is one strip
            if line[column] != "1":
                column += 1
                continue
            end = column
            while end + 1 < len(line) and line[end + 1] == "1":
                end += 1
            x = ((column + end + 1) / 2.0 - len(line) / 2.0) * cell
            y = (len(HOME_TAG_CELLS) / 2.0 - row - 0.5) * cell
            links.append(_flat_link(f"white_{row}_{column}", x, y, cells_z, 0.0, (end - column + 1) * cell, cell, white))
            column = end + 1
    return f"<?xml version='1.0'?><sdf version='1.6'><model name='{model_name}'><static>true</static>{''.join(links)}</model></sdf>"


def build_x_mat_sdf(
    model_name: str,
    size: float = 5.0,
    arm_width: float = 0.5,
    arm_inset: float = 0.25,
) -> str:
    """Build SDF for the recovery mat: a green X on a white square.

    The arms run along the diagonals and stop ``arm_inset`` short of the
    corners, as ``tools/synth_targets.py`` draws them.

    Args:
        model_name: Name for the model.
        size:       Side length of the mat in metres.
        arm_width:  Width of each arm of the X in metres.
        arm_inset:  Gap between an arm's tip and the mat corner in metres.
    """
    half = size / 2.0
    arm_length = 2.0 * (half * (2.0 ** 0.5) - arm_inset)
    if size <= 0.0 or arm_width <= 0.0 or arm_length <= 0.0:
        raise ValueError("x mat needs positive size, arm width and arm length")
    thickness = 0.02
    quarter_turn = 0.7853981633974483  # 45 degrees
    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        f"<link name='base'>"
        f"<pose>0 0 {thickness / 2:.4f} 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"{_material(_WHITE)}"
        f"</visual>"
        f"</link>"
        f"{_flat_link('arm_0', 0.0, 0.0, thickness + 0.002, quarter_turn, arm_length, arm_width, _X_GREEN)}"
        f"{_flat_link('arm_1', 0.0, 0.0, thickness + 0.004, -quarter_turn, arm_length, arm_width, _X_GREEN)}"
        f"</model>"
        f"</sdf>"
    )


def build_bucket_sdf(
    model_name: str,
    mass_kg: float = 1.5,
    diameter: float = 0.19,
    height: float = 0.20,
    handle_rise: float = 0.12,
) -> str:
    """Build SDF for the recovery bucket: a dynamic body with an upright handle.

    The model origin is on the ground at the centre of the bucket. The handle
    is two posts at the rim and a bar across them ``handle_rise`` above it, so
    a hook can pass under the bar. Body and handle are one rigid link with the
    mass and inertia of a solid cylinder of ``mass_kg``.

    Args:
        model_name:  Name for the model.
        mass_kg:     Total mass (the rules allow 1 to 2 kg).
        diameter:    Bucket diameter in metres.
        height:      Bucket height in metres.
        handle_rise: Height of the handle bar above the rim in metres.
    """
    if mass_kg <= 0.0:
        raise ValueError(f"bucket mass must be positive, got {mass_kg}")
    if diameter <= 0.0 or height <= 0.0 or handle_rise <= 0.0:
        raise ValueError("bucket diameter, height and handle rise must be positive")
    radius = diameter / 2.0
    rod = 0.004  # handle rod radius
    ixx = mass_kg * (3.0 * radius ** 2 + height ** 2) / 12.0
    izz = mass_kg * radius ** 2 / 2.0
    half_turn = 1.5707963267948966  # lays a cylinder along x

    def shape(kind: str, name: str, pose: str, geometry: str, extra: str = "") -> str:
        return f"<{kind} name='{name}'><pose>{pose}</pose><geometry>{geometry}</geometry>{extra}</{kind}>"

    parts = [
        ("body", f"0 0 {height / 2:.4f} 0 0 0", f"<cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder>"),
        ("handle_post_left", f"{-radius:.4f} 0 {height + handle_rise / 2:.4f} 0 0 0", f"<cylinder><radius>{rod}</radius><length>{handle_rise:.4f}</length></cylinder>"),
        ("handle_post_right", f"{radius:.4f} 0 {height + handle_rise / 2:.4f} 0 0 0", f"<cylinder><radius>{rod}</radius><length>{handle_rise:.4f}</length></cylinder>"),
        ("handle_bar", f"0 0 {height + handle_rise:.4f} 0 {half_turn} 0", f"<cylinder><radius>{rod}</radius><length>{diameter:.4f}</length></cylinder>"),
    ]
    collisions = "".join(shape("collision", name, pose, geometry) for name, pose, geometry in parts)
    visuals = "".join(
        shape("visual", name, pose, geometry, _material(_BUCKET_ORANGE if name == "body" else _PLATE_BLACK))
        for name, pose, geometry in parts
    )
    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>false</static>"
        f"<link name='link'>"
        f"<pose>0 0 0 0 0 0</pose>"
        f"<inertial>"
        f"<pose>0 0 {height / 2:.4f} 0 0 0</pose>"
        f"<mass>{mass_kg}</mass>"
        f"<inertia><ixx>{ixx:.10g}</ixx><iyy>{ixx:.10g}</iyy><izz>{izz:.10g}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
        f"</inertial>"
        f"{collisions}"
        f"{visuals}"
        f"</link>"
        f"</model>"
        f"</sdf>"
    )


def build_payload_sdf(model_name: str, mass_kg: float = 0.5, size=(0.15, 0.15, 0.10)) -> str:
    """Build SDF for a drop or delivery package: a dynamic box resting on the ground.

    Args:
        model_name: Name for the model.
        mass_kg:    Package mass.
        size:       (x, y, z) side lengths in metres.
    """
    sx, sy, sz = (float(v) for v in size)
    if mass_kg <= 0.0 or min(sx, sy, sz) <= 0.0:
        raise ValueError("payload mass and size must be positive")
    ixx = mass_kg * (sy ** 2 + sz ** 2) / 12.0
    iyy = mass_kg * (sx ** 2 + sz ** 2) / 12.0
    izz = mass_kg * (sx ** 2 + sy ** 2) / 12.0
    box = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>false</static>"
        f"<link name='link'>"
        f"<pose>0 0 {sz / 2:.4f} 0 0 0</pose>"
        f"<inertial>"
        f"<mass>{mass_kg}</mass>"
        f"<inertia><ixx>{ixx:.10g}</ixx><iyy>{iyy:.10g}</iyy><izz>{izz:.10g}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
        f"</inertial>"
        f"<collision name='collision'>{box}</collision>"
        f"<visual name='visual'>{box}{_material('0.55 0.4 0.25 1')}</visual>"
        f"</link>"
        f"</model>"
        f"</sdf>"
    )


if __name__ == "__main__":
    # Example usage: spawn a target at the origin
    sdf = build_red_white_target_sdf("test_target", size=5.0)
    spawn_sdf(sdf, "test_target", x=5, y=5, z=1)
#!/usr/bin/env python3
"""
altium_ascii_to_kicad_hardcoded_fixed.py

Hardcoded converter for old Protel/Altium ASCII pipe-delimited PCB/footprint
records, VERSION=5.01 style, into a KiCad 8/9 .kicad_mod footprint.

This script intentionally has no argparse and no command-line interface.
Edit the constants in the configuration section, put the source .pcbdoc/.txt
beside this script, then run:

    python3 altium_ascii_to_kicad_hardcoded_fixed.py

The uploaded P-VFBGA-80 file needs two important handling rules:

1. Altium PCB coordinates and KiCad footprint coordinates are mirrored in Y for
   this export, so FLIP_Y_AXIS defaults to True. Without this, A1/top-left
   features appear vertically flipped in KiCad.

2. This file stores many drawn circular shapes as Region records with two arc
   segments. A simple converter that skips KINDn=1 regions loses those shapes.
   This version approximates arc-based Region records into filled fp_poly items.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# =============================================================================
# Hardcoded configuration
# =============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent

INPUT_FILE = SCRIPT_DIR / "TF-SMD_TF-PUSH.pcbdoc"
OUTPUT_FILE = SCRIPT_DIR / "TF-SMD_TF-PUSH.kicad_mod"

FOOTPRINT_NAME = "P-VFBGA-80_L7.0-W7.0-R10-C10-P0.65-TL"
AUTO_NAME_FROM_COMPONENT = True

# This is required for this uploaded footprint: A1/pin-1 should remain top-left.
FLIP_Y_AXIS = True

# Convert Region records containing arc segments. The uploaded file uses this
# form for 80 circular mechanical regions plus the pin-1 marker region.
INCLUDE_ARC_REGIONS = True
ARC_APPROX_STEP_DEGREES = 10.0

GENERATOR_NAME = "altium_ascii_to_kicad_hardcoded_fixed"
DEFAULT_TEXT_OFFSET_MM = 6.5
DEFAULT_GRAPHIC_WIDTH_MIL = "4mil"
DUPLICATE_POINT_TOLERANCE_MM = 0.001
MIN_POLYGON_UNIQUE_POINTS = 3


# =============================================================================
# Type aliases
# =============================================================================

Record = Dict[str, str]
Point = Tuple[float, float]


# =============================================================================
# Low-level helpers
# =============================================================================

def kicad_string(value: object) -> str:
    """Escape text for a quoted KiCad S-expression string."""
    assert value is not None, "value must not be None"
    text = str(value)
    text = text.replace("\\", "\\\\")
    text = text.replace('"', '\\"')
    text = text.replace("\r", " ")
    text = text.replace("\n", " ")
    return text


def mil_to_mm(value: object) -> float:
    """Convert a numeric Altium value in mils into millimetres."""
    assert value is not None, "value must not be None"
    text = str(value).strip().lower()
    if text == "":
        return 0.0
    text = text.replace("mil", "").strip()
    if text == "":
        return 0.0
    return float(text) * 0.0254


def read_float(record: Record, key: str, default: float = 0.0) -> float:
    """Read a floating-point field from one parsed record."""
    assert isinstance(record, dict), "record must be a dictionary"
    assert key, "key must not be blank"
    raw_value = record.get(key, default)
    if raw_value is None:
        return float(default)
    text = str(raw_value).strip()
    if text == "":
        return float(default)
    return float(text)


def read_mil(record: Record, key: str, default: object = 0) -> float:
    """Read one mil-based field from a record and convert it to millimetres."""
    assert isinstance(record, dict), "record must be a dictionary"
    assert key, "key must not be blank"
    return mil_to_mm(record.get(key, default))


def bool_field(record: Record, key: str, default: bool = False) -> bool:
    """Read an Altium-style boolean field."""
    assert isinstance(record, dict), "record must be a dictionary"
    assert key, "key must not be blank"
    raw_value = record.get(key)
    if raw_value is None:
        return default
    return str(raw_value).strip().upper() in {"TRUE", "YES", "1", "ON"}


def normalize_layer_name(layer: object) -> str:
    """Normalize layer spelling before lookup."""
    assert layer is not None, "layer must not be None"
    return str(layer).strip().upper().replace(" ", "").replace("_", "")


def map_layer(altium_layer: object) -> str:
    """Map common Protel/Altium layer names to KiCad footprint layers."""
    assert altium_layer is not None, "altium_layer must not be None"
    mapping = {
        "TOP": "F.Cu",
        "TOPLAYER": "F.Cu",
        "TOPOVERLAY": "F.SilkS",
        "TOPPASTE": "F.Paste",
        "TOPSOLDER": "F.Mask",
        "BOTTOM": "B.Cu",
        "BOTTOMLAYER": "B.Cu",
        "BOTTOMOVERLAY": "B.SilkS",
        "BOTTOMPASTE": "B.Paste",
        "BOTTOMSOLDER": "B.Mask",
        "MECHANICAL1": "F.Fab",
        "MECHANICAL2": "F.Fab",
        "MECHANICAL3": "F.Fab",
        "MECHANICAL4": "F.Fab",
        "MECHANICAL5": "F.Fab",
        "MECHANICAL6": "F.CrtYd",
        "MECHANICAL7": "F.CrtYd",
        "MECHANICAL8": "F.Fab",
        "MECHANICAL9": "F.Fab",
        "MECHANICAL10": "User.1",
        "MECHANICAL11": "User.2",
        "MECHANICAL12": "User.3",
        "MECHANICAL13": "User.4",
        "MECHANICAL14": "User.5",
        "MECHANICAL15": "User.6",
        "MECHANICAL16": "User.7",
        "KEEPOUT": "F.CrtYd",
        "MULTI-LAYER": "*.Cu",
        "MULTILAYER": "*.Cu",
        "DRILLDRAWING": "Dwgs.User",
        "BOARDOUTLINE": "Edge.Cuts",
    }
    normalized = normalize_layer_name(altium_layer)
    return mapping.get(normalized, str(altium_layer).strip())


def almost_same_point(a: Point, b: Point, tol: float = DUPLICATE_POINT_TOLERANCE_MM) -> bool:
    """Return True when two local KiCad points are effectively identical."""
    assert len(a) == 2, "point a must have two coordinates"
    assert len(b) == 2, "point b must have two coordinates"
    assert tol >= 0.0, "tolerance must be non-negative"
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def positive_sweep_degrees(start_degrees: float, end_degrees: float) -> float:
    """Return a positive counter-clockwise sweep from start to end."""
    sweep = (end_degrees - start_degrees) % 360.0
    if sweep < 0.1:
        return 360.0
    return sweep


def convert_rotation_degrees(rotation: float) -> float:
    """Mirror rotation when the footprint Y axis is mirrored."""
    if FLIP_Y_AXIS:
        return (-rotation) % 360.0
    return rotation % 360.0


def add_point_if_new(points: List[Point], point: Point) -> None:
    """Append a point only when it is not a duplicate of the previous point."""
    assert isinstance(points, list), "points must be a mutable list"
    assert len(point) == 2, "point must contain two coordinates"
    if not points or not almost_same_point(points[-1], point):
        points.append(point)


def count_parentheses(text: str) -> Tuple[int, int]:
    """Count KiCad S-expression parentheses for a simple sanity check."""
    assert isinstance(text, str), "text must be a string"
    return text.count("("), text.count(")")


# =============================================================================
# Parser
# =============================================================================

def parse_records(text: str) -> List[Record]:
    """Parse pipe-delimited Altium ASCII lines into dictionaries."""
    assert isinstance(text, str), "text must be a string"
    records: List[Record] = []

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line.startswith("|RECORD="):
            continue

        line_body = line[1:-1] if line.endswith("|") else line[1:]
        record: Record = {}

        for field in line_body.split("|"):
            if "=" not in field:
                continue
            key, value = field.split("=", 1)
            key = key.strip()
            if key:
                record[key] = value.strip()

        if record:
            records.append(record)

    return records


# =============================================================================
# Converter
# =============================================================================

class FootprintConverter:
    """Convert parsed Altium records into one KiCad footprint."""

    def __init__(self, records: Sequence[Record], footprint_name: str) -> None:
        assert len(records) > 0, "records must not be empty"
        assert footprint_name.strip(), "footprint_name must not be blank"
        self.records = list(records)
        self.name = footprint_name.strip()
        self.component = self._find_component()
        self.origin_x_mm = self._get_origin_x_mm()
        self.origin_y_mm = self._get_origin_y_mm()

    def _records_of_type(self, record_type: str) -> Iterable[Record]:
        """Yield all records matching one Altium record type."""
        assert record_type.strip(), "record_type must not be blank"
        for record in self.records:
            if record.get("RECORD") == record_type:
                yield record

    def _find_component(self) -> Optional[Record]:
        """Find the first Component record, if present."""
        for record in self.records:
            if record.get("RECORD") == "Component":
                return record
        return None

    def _get_origin_x_mm(self) -> float:
        """Use Component X first; fall back to pad centroid."""
        if self.component is not None and "X" in self.component:
            return read_mil(self.component, "X")
        pad_x_values = [read_mil(pad, "X") for pad in self._records_of_type("Pad") if "X" in pad]
        if pad_x_values:
            return sum(pad_x_values) / len(pad_x_values)
        return 0.0

    def _get_origin_y_mm(self) -> float:
        """Use Component Y first; fall back to pad centroid."""
        if self.component is not None and "Y" in self.component:
            return read_mil(self.component, "Y")
        pad_y_values = [read_mil(pad, "Y") for pad in self._records_of_type("Pad") if "Y" in pad]
        if pad_y_values:
            return sum(pad_y_values) / len(pad_y_values)
        return 0.0

    def local_x(self, altium_x: object) -> float:
        """Convert absolute Altium X into local KiCad X."""
        assert altium_x is not None, "altium_x must not be None"
        return mil_to_mm(altium_x) - self.origin_x_mm

    def local_y(self, altium_y: object) -> float:
        """Convert absolute Altium Y into local KiCad Y, with optional mirroring."""
        assert altium_y is not None, "altium_y must not be None"
        y_mm = mil_to_mm(altium_y)
        if FLIP_Y_AXIS:
            return self.origin_y_mm - y_mm
        return y_mm - self.origin_y_mm

    def local_point(self, altium_x: object, altium_y: object) -> Point:
        """Convert an absolute Altium point into local KiCad coordinates."""
        assert altium_x is not None, "altium_x must not be None"
        assert altium_y is not None, "altium_y must not be None"
        return (self.local_x(altium_x), self.local_y(altium_y))

    def local_polar_point(self, cx_value: object, cy_value: object, radius_value: object, angle: float) -> Point:
        """Compute an Altium polar point and transform it into KiCad coordinates."""
        assert cx_value is not None, "cx_value must not be None"
        assert cy_value is not None, "cy_value must not be None"
        assert radius_value is not None, "radius_value must not be None"
        cx_mm = mil_to_mm(cx_value)
        cy_mm = mil_to_mm(cy_value)
        radius_mm = mil_to_mm(radius_value)
        angle_rad = math.radians(angle)
        x_mm = cx_mm + radius_mm * math.cos(angle_rad)
        y_mm = cy_mm + radius_mm * math.sin(angle_rad)
        if FLIP_Y_AXIS:
            return (x_mm - self.origin_x_mm, self.origin_y_mm - y_mm)
        return (x_mm - self.origin_x_mm, y_mm - self.origin_y_mm)

    def choose_attr_line(self) -> Optional[str]:
        """Choose KiCad footprint attribute based on pad hole presence."""
        has_smd = False
        has_tht = False
        for pad in self._records_of_type("Pad"):
            if read_mil(pad, "HOLESIZE", 0) > 0.0:
                has_tht = True
            else:
                has_smd = True
        if has_smd:
            return "  (attr smd)"
        if has_tht:
            return "  (attr through_hole)"
        return None

    def convert(self) -> str:
        """Build the complete KiCad footprint text."""
        lines: List[str] = []
        lines.append(f'(footprint "{kicad_string(self.name)}"')
        lines.append("  (version 20240108)")
        lines.append(f'  (generator "{kicad_string(GENERATOR_NAME)}")')
        lines.append('  (layer "F.Cu")')
        lines.append('  (descr "Converted from Protel/Altium ASCII PCB/footprint records")')

        attr_line = self.choose_attr_line()
        if attr_line is not None:
            lines.append(attr_line)

        lines.extend(self._header_texts())
        lines.extend(self._pads())
        lines.extend(self._tracks())
        lines.extend(self._arcs())
        lines.extend(self._regions())
        lines.extend(self._texts())
        lines.append(")")
        return "\n".join(lines) + "\n"

    def _header_texts(self) -> List[str]:
        """Generate default KiCad reference/value text."""
        return [
            f'  (fp_text reference "REF**" (at 0 {-DEFAULT_TEXT_OFFSET_MM:.4f}) (layer "F.SilkS")',
            '    (effects (font (size 1 1) (thickness 0.15)))',
            "  )",
            f'  (fp_text value "{kicad_string(self.name)}" (at 0 {DEFAULT_TEXT_OFFSET_MM:.4f}) (layer "F.Fab")',
            '    (effects (font (size 1 1) (thickness 0.15)))',
            "  )",
        ]

    def _pad_shape(self, record: Record, sx: float, sy: float) -> str:
        """Map an Altium pad shape to a KiCad pad shape."""
        assert isinstance(record, dict), "record must be a dictionary"
        assert sx >= 0.0, "sx must be non-negative"
        assert sy >= 0.0, "sy must be non-negative"
        shape = record.get("SHAPE", "RECTANGLE").strip().upper()
        if shape in {"RECTANGLE", "SQUARE"}:
            return "rect"
        if shape in {"ROUND", "CIRCLE", "CIRCULAR"}:
            return "circle" if abs(sx - sy) <= 0.001 else "oval"
        if shape in {"OVAL", "ROUNDED"}:
            return "oval"
        return "roundrect"

    def _smd_layers_for_pad(self, record: Record) -> str:
        """Return the KiCad layers for one SMD pad."""
        assert isinstance(record, dict), "record must be a dictionary"
        source_layer = map_layer(record.get("LAYER", "TOP"))
        if source_layer.startswith("B."):
            return '"B.Cu" "B.Mask" "B.Paste"'
        return '"F.Cu" "F.Mask" "F.Paste"'

    def _pads(self) -> List[str]:
        """Convert Pad records."""
        lines: List[str] = []
        for pad in self._records_of_type("Pad"):
            name = kicad_string(pad.get("NAME", ""))
            x, y = self.local_point(pad.get("X", 0), pad.get("Y", 0))
            sx = read_mil(pad, "XSIZE", 0)
            sy = read_mil(pad, "YSIZE", 0)
            rotation = convert_rotation_degrees(read_float(pad, "ROTATION", 0.0))
            hole = read_mil(pad, "HOLESIZE", 0)
            shape = self._pad_shape(pad, sx, sy)

            if sx <= 0.0 or sy <= 0.0:
                continue

            if hole > 0.0:
                lines.append(
                    f'  (pad "{name}" thru_hole {shape} '
                    f'(at {x:.4f} {y:.4f} {rotation:.4f}) '
                    f'(size {sx:.4f} {sy:.4f}) '
                    f'(drill {hole:.4f}) '
                    f'(layers "*.Cu" "*.Mask"))'
                )
            else:
                layers = self._smd_layers_for_pad(pad)
                lines.append(
                    f'  (pad "{name}" smd {shape} '
                    f'(at {x:.4f} {y:.4f} {rotation:.4f}) '
                    f'(size {sx:.4f} {sy:.4f}) '
                    f'(layers {layers}))'
                )
        return lines

    def _tracks(self) -> List[str]:
        """Convert Track records into fp_line items."""
        lines: List[str] = []
        for track in self._records_of_type("Track"):
            layer = map_layer(track.get("LAYER", ""))
            if layer == "":
                continue
            x1, y1 = self.local_point(track.get("X1", 0), track.get("Y1", 0))
            x2, y2 = self.local_point(track.get("X2", 0), track.get("Y2", 0))
            width = read_mil(track, "WIDTH", DEFAULT_GRAPHIC_WIDTH_MIL)
            if width <= 0.0:
                continue
            lines.append(
                f'  (fp_line (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) '
                f'(stroke (width {width:.4f}) (type solid)) '
                f'(layer "{kicad_string(layer)}"))'
            )
        return lines

    def _arcs(self) -> List[str]:
        """Convert standalone Arc records."""
        lines: List[str] = []
        for arc in self._records_of_type("Arc"):
            layer = map_layer(arc.get("LAYER", ""))
            if layer == "":
                continue
            cx, cy = self.local_point(arc.get("LOCATION.X", 0), arc.get("LOCATION.Y", 0))
            radius = read_mil(arc, "RADIUS", 0)
            start_angle = read_float(arc, "STARTANGLE", 0.0)
            end_angle = read_float(arc, "ENDANGLE", 360.0)
            width = read_mil(arc, "WIDTH", DEFAULT_GRAPHIC_WIDTH_MIL)
            if radius <= 0.0 or width <= 0.0:
                continue

            sweep = positive_sweep_degrees(start_angle, end_angle)
            if abs(sweep - 360.0) < 0.1:
                end_x = cx + radius
                end_y = cy
                lines.append(
                    f'  (fp_circle (center {cx:.4f} {cy:.4f}) '
                    f'(end {end_x:.4f} {end_y:.4f}) '
                    f'(stroke (width {width:.4f}) (type solid)) '
                    f'(fill none) '
                    f'(layer "{kicad_string(layer)}"))'
                )
            else:
                start_point = self.local_polar_point(arc.get("LOCATION.X", 0), arc.get("LOCATION.Y", 0), arc.get("RADIUS", 0), start_angle)
                mid_point = self.local_polar_point(arc.get("LOCATION.X", 0), arc.get("LOCATION.Y", 0), arc.get("RADIUS", 0), start_angle + sweep / 2.0)
                end_point = self.local_polar_point(arc.get("LOCATION.X", 0), arc.get("LOCATION.Y", 0), arc.get("RADIUS", 0), start_angle + sweep)
                lines.append(
                    f'  (fp_arc (start {start_point[0]:.4f} {start_point[1]:.4f}) '
                    f'(mid {mid_point[0]:.4f} {mid_point[1]:.4f}) '
                    f'(end {end_point[0]:.4f} {end_point[1]:.4f}) '
                    f'(stroke (width {width:.4f}) (type solid)) '
                    f'(layer "{kicad_string(layer)}"))'
                )
        return lines

    def _region_vertex_count(self, region: Record) -> int:
        """Find the number of VX/VY entries in one Region record."""
        assert isinstance(region, dict), "region must be a dictionary"
        explicit = region.get("MAINCONTOURVERTEXCOUNT")
        if explicit is not None and str(explicit).strip().isdigit():
            return int(str(explicit).strip())
        count = 0
        while f"VX{count}" in region and f"VY{count}" in region:
            count += 1
        return count

    def _region_points(self, region: Record) -> List[Point]:
        """Convert line and arc segments in a Region record into polygon points."""
        assert isinstance(region, dict), "region must be a dictionary"
        vertex_count = self._region_vertex_count(region)
        points: List[Point] = []

        for index in range(vertex_count):
            vx_key = f"VX{index}"
            vy_key = f"VY{index}"
            kind = region.get(f"KIND{index}", "0").strip()
            if vx_key not in region or vy_key not in region:
                continue

            if kind == "1" and INCLUDE_ARC_REGIONS:
                cx_key = f"CX{index}"
                cy_key = f"CY{index}"
                radius_key = f"R{index}"
                start_key = f"SA{index}"
                end_key = f"EA{index}"
                if cx_key not in region or cy_key not in region or radius_key not in region:
                    add_point_if_new(points, self.local_point(region[vx_key], region[vy_key]))
                    continue

                start_angle = read_float(region, start_key, 0.0)
                end_angle = read_float(region, end_key, 0.0)
                sweep = positive_sweep_degrees(start_angle, end_angle)
                step_count = max(2, int(math.ceil(sweep / ARC_APPROX_STEP_DEGREES)))
                for step in range(step_count + 1):
                    angle = start_angle + sweep * step / step_count
                    point = self.local_polar_point(region[cx_key], region[cy_key], region[radius_key], angle)
                    add_point_if_new(points, point)
            else:
                add_point_if_new(points, self.local_point(region[vx_key], region[vy_key]))

        return self._clean_polygon(points)

    def _clean_polygon(self, vertices: Sequence[Point]) -> List[Point]:
        """Remove duplicate vertices and close the polygon."""
        assert vertices is not None, "vertices must not be None"
        if len(vertices) < MIN_POLYGON_UNIQUE_POINTS:
            return []

        cleaned: List[Point] = []
        for point in vertices:
            add_point_if_new(cleaned, point)

        if len(cleaned) < MIN_POLYGON_UNIQUE_POINTS:
            return []

        if almost_same_point(cleaned[0], cleaned[-1]):
            closed = cleaned
        else:
            closed = cleaned + [cleaned[0]]

        unique = {(round(x, 3), round(y, 3)) for x, y in closed[:-1]}
        if len(unique) < MIN_POLYGON_UNIQUE_POINTS:
            return []
        return closed

    def _regions(self) -> List[str]:
        """Convert Region records into filled fp_poly items."""
        lines: List[str] = []
        for region in self._records_of_type("Region"):
            layer = map_layer(region.get("LAYER", ""))
            if layer == "":
                continue

            has_arc_segment = any(
                region.get(f"KIND{index}") == "1"
                for index in range(self._region_vertex_count(region))
            )
            if has_arc_segment and not INCLUDE_ARC_REGIONS:
                continue

            polygon_points = self._region_points(region)
            if len(polygon_points) < 4:
                continue

            point_text = " ".join(f"(xy {x:.4f} {y:.4f})" for x, y in polygon_points)
            lines.append(
                f'  (fp_poly (pts {point_text}) '
                f'(stroke (width 0) (type solid)) '
                f'(fill solid) '
                f'(layer "{kicad_string(layer)}"))'
            )
        return lines

    def _texts(self) -> List[str]:
        """Convert non-designator Text records."""
        lines: List[str] = []
        for text_record in self._records_of_type("Text"):
            layer = map_layer(text_record.get("LAYER", ""))
            if layer == "":
                continue
            if bool_field(text_record, "DESIGNATOR") or bool_field(text_record, "COMMENT"):
                continue

            text = text_record.get("TEXT", "").strip()
            if text == "":
                continue

            x, y = self.local_point(text_record.get("X", 0), text_record.get("Y", 0))
            height = read_mil(text_record, "HEIGHT", "50mil")
            thickness = read_mil(text_record, "WIDTH", "6mil")
            rotation = convert_rotation_degrees(read_float(text_record, "ROTATION", 0.0))
            if height <= 0.0 or thickness <= 0.0:
                continue

            if bool_field(text_record, "MIRROR") and layer.startswith("F."):
                layer = "B." + layer[2:]

            lines.append(
                f'  (fp_text user "{kicad_string(text)}" '
                f'(at {x:.4f} {y:.4f} {rotation:.4f}) '
                f'(layer "{kicad_string(layer)}")'
            )
            lines.append(
                f'    (effects (font (size {height:.4f} {height:.4f}) '
                f'(thickness {thickness:.4f})))'
            )
            lines.append("  )")
        return lines


# =============================================================================
# Main flow
# =============================================================================

def choose_footprint_name(records: Sequence[Record]) -> str:
    """Choose footprint name from Component/PATTERN, otherwise use the constant."""
    assert len(records) > 0, "records must not be empty"
    if AUTO_NAME_FROM_COMPONENT:
        for record in records:
            if record.get("RECORD") == "Component" and record.get("PATTERN", "").strip():
                return record["PATTERN"].strip()
    return FOOTPRINT_NAME


def validate_generated_text(text: str) -> None:
    """Run cheap safety checks before writing the KiCad footprint."""
    assert isinstance(text, str), "text must be a string"
    open_count, close_count = count_parentheses(text)
    if open_count != close_count:
        raise ValueError(f"Generated S-expression is unbalanced: {open_count} '(' vs {close_count} ')'")
    if "(footprint " not in text:
        raise ValueError("Generated output does not contain a KiCad footprint root node")
    if text.count("(pad ") == 0:
        raise ValueError("Generated output contains no pads")


def run_conversion() -> Path:
    """Read the hardcoded input file and write the hardcoded KiCad output file."""
    assert INPUT_FILE.name != "", "INPUT_FILE must have a filename"
    assert OUTPUT_FILE.name != "", "OUTPUT_FILE must have a filename"

    if not INPUT_FILE.exists():
        raise FileNotFoundError(f"Input file not found: {INPUT_FILE}")

    source_text = INPUT_FILE.read_text(encoding="utf-8", errors="ignore")
    records = parse_records(source_text)
    if not records:
        raise ValueError("No |RECORD=...| records were found in the input file")

    footprint_name = choose_footprint_name(records)
    converter = FootprintConverter(records, footprint_name)
    output_text = converter.convert()
    validate_generated_text(output_text)

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_FILE.write_text(output_text, encoding="utf-8")

    print(f"Saved: {OUTPUT_FILE}")
    print(f"Records parsed: {len(records)}")
    print(f"Pads emitted: {output_text.count('(pad ')}")
    print(f"Lines emitted: {output_text.count('(fp_line')}")
    print(f"Standalone circles emitted: {output_text.count('(fp_circle')}")
    print(f"Polygons emitted: {output_text.count('(fp_poly')}")
    print(f"Footprint name: {footprint_name}")
    print(f"Origin: ({converter.origin_x_mm:.3f}, {converter.origin_y_mm:.3f}) mm")
    print(f"Y axis flipped: {FLIP_Y_AXIS}")

    return OUTPUT_FILE


def main() -> None:
    """Program entry point; this deliberately ignores command-line arguments."""
    run_conversion()


if __name__ == "__main__":
    main()

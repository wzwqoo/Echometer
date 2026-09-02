"""
FreeCAD Macro: Ultrasonic Conical Horn for SPU0410LR5H-QB
Generates a parametric conical horn optimized for 40-50 kHz bat detection.
Run in FreeCAD: Macro > Macros > Execute (or paste in Python console)
"""

import FreeCAD as App
import Part

# ============================================================
# PARAMETERS - Adjust these to your needs
# ============================================================
THROAT_ID = 0.30      # Inner diameter at mic end (mm) - slightly larger than 0.25mm port
MOUTH_ID = 28.0       # Inner diameter at open end (mm) - ~3.5x wavelength at 45kHz
LENGTH = 50.0         # Total horn length (mm)
WALL_THICKNESS = 0.5  # Wall thickness (mm) - keep rigid at 40-50 kHz

# Mounting features
MOUNT_LEN = 5.0       # Length of cylinder behind mic for PCB mounting
MOUNT_OD = 6.0        # Outer diameter of mounting cylinder
THROAT_TUBE_LEN = 2.0 # Small tube extending behind to fit over mic port
THROAT_TUBE_OD = 1.0  # Outer diameter of throat tube

# Rim stiffening
RIM_WIDTH = 2.0       # How far rim extends past mouth OD
RIM_THICKNESS = 1.0   # Thickness of rim ring

# ============================================================
# GEOMETRY GENERATION
# ============================================================

doc = App.newDocument("UltrasonicConeHorn")

# --- Main conical horn body (hollow) ---
# Profile in X-Y plane: X=radius, Y=length axis
# Revolve around Y axis
p1 = App.Vector(THROAT_ID/2, 0, 0)                           # throat inner
p2 = App.Vector(MOUTH_ID/2, LENGTH, 0)                       # mouth inner
p3 = App.Vector(MOUTH_ID/2 + WALL_THICKNESS, LENGTH, 0)    # mouth outer
p4 = App.Vector(THROAT_ID/2 + WALL_THICKNESS, 0, 0)          # throat outer

wire = Part.Wire([
    Part.makeLine(p1, p2),
    Part.makeLine(p2, p3),
    Part.makeLine(p3, p4),
    Part.makeLine(p4, p1)
])
face = Part.Face(wire)
cone = face.revolve(App.Vector(0,0,0), App.Vector(0,1,0), 360)

# --- Mounting cylinder (solid, behind mic) ---
mount = Part.makeCylinder(MOUNT_OD/2, MOUNT_LEN,
                          App.Vector(0, -MOUNT_LEN, 0),
                          App.Vector(0, 1, 0))

# --- Throat tube (press-fit over mic port) ---
# This is a thin tube extending backward from the throat
throat_tube = Part.makeCylinder(THROAT_TUBE_OD/2, THROAT_TUBE_LEN,
                                App.Vector(0, -THROAT_TUBE_LEN, 0),
                                App.Vector(0, 1, 0))

# --- Rim ring at mouth (for stiffness and mounting) ---
rim_outer_rad = (MOUTH_ID/2 + WALL_THICKNESS) + RIM_WIDTH
rim = Part.makeCylinder(rim_outer_rad, RIM_THICKNESS,
                        App.Vector(0, LENGTH, 0),
                        App.Vector(0, 1, 0))
# Cut out the inner part so it's just a ring
rim_inner_rad = MOUTH_ID/2 + WALL_THICKNESS - 0.01
rim_cutter = Part.makeCylinder(rim_inner_rad, RIM_THICKNESS + 0.02,
                               App.Vector(0, LENGTH - 0.01, 0),
                               App.Vector(0, 1, 0))
rim_ring = rim.cut(rim_cutter)

# --- Fuse all parts into one solid ---
body = cone.fuse(mount)
body = body.fuse(throat_tube)
body = body.fuse(rim_ring)
body = body.removeSplitter()

# --- Add to document ---
horn_obj = doc.addObject("Part::Feature", "HornBody")
horn_obj.Shape = body

doc.recompute()

# ============================================================
# INFO & EXPORT INSTRUCTIONS
# ============================================================
print("="*60)
print("ULTRASONIC CONICAL HORN GENERATED")
print("="*60)
print(f"Throat inner diameter: {THROAT_ID} mm")
print(f"Mouth inner diameter:  {MOUTH_ID} mm")
print(f"Total length:          {LENGTH} mm")
print(f"Wall thickness:        {WALL_THICKNESS} mm")
print(f"Flare half-angle:      {App.Units.parseQuantity(str((MOUTH_ID/2 - THROAT_ID/2) / LENGTH))} rad")
print(f"Solid volume:          {body.Volume:.2f} mm^3")
print(f"Mass (brass, ~8.4g/cc): {body.Volume * 8.4e-3:.2f} g")
print(f"Mass (ABS, ~1.05g/cc):  {body.Volume * 1.05e-3:.2f} g")
print("="*60)
print("\nTo export for 3D printing:")
print("1. Select 'HornBody' in the tree view")
print("2. File > Export > Mesh (.stl, .3mf, .obj)")
print("3. For resin print: use 25 micron layers, orient vertically")
print("4. For FDM: use 0.12mm layers, 100% infill, print slow")
print("="*60)

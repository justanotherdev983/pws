import math
import FreeCAD as App
import Part

# ----------------------------
# RC Airplane Parameters
# ----------------------------
DOC_NAME = "RC_Plane_Complete"
EXPORT_STEP = True
EXPORT_PATH = App.getUserAppDataDir() + "rc_plane_complete.step"

# Wing - NACA 2412 (2% camber, 40% max camber, 12% thickness)
NACA_CODE = "2412"
SPAN = 900.0              # full wingspan, mm (90 cm)
SEMI_SPAN = SPAN * 0.5
CHORD_ROOT = 120.0        # mm (narrower wing for better proportions)
TAPER_RATIO = 0.65        # tip/root ratio
CHORD_TIP = CHORD_ROOT * TAPER_RATIO
SWEEP_LE = 8.0            # degrees
DIHEDRAL = 4.0            # degrees
WASHOUT_TIP = 2.5         # degrees (reduces tip stall)
AF_POINTS = 121           # airfoil resolution

# Fuselage
FUSE_LEN = 600.0          # mm (60 cm)
FUSE_RADIUS = 27.5        # mm (2.75 cm radius - circular cross-section)
NOSE_LEN = 100.0          # mm (scaled proportionally)

# Horizontal Stabilizer
HTP_SPAN = 300.0          # mm (scaled proportionally)
HTP_SEMI = HTP_SPAN * 0.5
HTP_CHORD_ROOT = 70.0
HTP_CHORD_TIP = 50.0
HTP_AIRFOIL = "0012"      # symmetric

# Vertical Stabilizer
VTP_HEIGHT = 90.0
VTP_CHORD_ROOT = 85.0
VTP_CHORD_TIP = 45.0
VTP_AIRFOIL = "0012"

# Motor Mount
MOTOR_DIAM = 24.0         # mm (scaled)
FIREWALL_THICKNESS = 2.0

# Placement
WING_X_AT_ROOT = 230.0    # wing LE from nose (scaled)
WING_Z = -15.0            # slightly below fuselage centerline
TAIL_X = FUSE_LEN - 120.0
TAIL_Z = 0.0

# ----------------------------
# NACA 4-digit Airfoil Generator
# ----------------------------
def naca4_points(code, n=121):
    """
    Generate NACA 4-digit airfoil coordinates.
    Returns list of (x, z) tuples forming closed loop.
    code: string like "4412" (m, p, t)
    """
    m = int(code[0]) / 100.0      # max camber
    p = int(code[1]) / 10.0       # position of max camber
    t = int(code[2:]) / 100.0     # thickness
    
    # Cosine spacing for better leading edge
    beta = [i * math.pi / (n - 1) for i in range(n)]
    x = [(1.0 - math.cos(b)) / 2.0 for b in beta]
    
    xu, zu, xl, zl = [], [], [], []
    
    for xi in x:
        # Thickness distribution
        yt = 5 * t * (0.2969 * math.sqrt(xi) - 0.1260 * xi - 0.3516 * xi**2 
                      + 0.2843 * xi**3 - 0.1015 * xi**4)
        
        # Mean camber line
        if xi < p and p > 0:
            yc = (m / p**2) * (2 * p * xi - xi**2)
            dyc_dx = (2 * m / p**2) * (p - xi)
        elif p > 0:
            yc = (m / (1 - p)**2) * ((1 - 2 * p) + 2 * p * xi - xi**2)
            dyc_dx = (2 * m / (1 - p)**2) * (p - xi)
        else:
            yc = 0.0
            dyc_dx = 0.0
        
        theta = math.atan(dyc_dx)
        
        xu.append(xi - yt * math.sin(theta))
        zu.append(yc + yt * math.cos(theta))
        xl.append(xi + yt * math.sin(theta))
        zl.append(yc - yt * math.cos(theta))
    
    # Create closed loop: upper surface + lower surface reversed
    x_coords = xu + xl[::-1]
    z_coords = zu + zl[::-1]
    
    return list(zip(x_coords, z_coords))

# ----------------------------
# Helper Functions
# ----------------------------
def make_airfoil_wire(code, chord, n=121):
    """Create closed wire from NACA airfoil, scaled to chord length."""
    points = naca4_points(code, n)
    pts = [App.Vector(x * chord, 0, z * chord) for x, z in points]
    
    # Create B-spline through points
    spline = Part.BSplineCurve()
    spline.interpolate(pts, False)
    edge = spline.toShape()
    
    # Close the wire
    if edge.Vertexes[0].Point.distanceToPoint(edge.Vertexes[-1].Point) > 0.01:
        closing_edge = Part.LineSegment(edge.Vertexes[-1].Point, 
                                        edge.Vertexes[0].Point).toShape()
        wire = Part.Wire([edge, closing_edge])
    else:
        wire = Part.Wire([edge])
    
    return wire

def transform_shape(shape, pos, rot_x=0, rot_y=0, rot_z=0):
    """Transform shape: rotate then translate."""
    s = shape.copy()
    rot = App.Rotation(App.Vector(1,0,0), rot_x)
    rot = rot.multiply(App.Rotation(App.Vector(0,1,0), rot_y))
    rot = rot.multiply(App.Rotation(App.Vector(0,0,1), rot_z))
    s.Placement = App.Placement(pos, rot)
    return s

def wing_section(y, chord, sweep, dihedral, washout, ref_x):
    """Create wing section at spanwise position y."""
    wire = make_airfoil_wire(NACA_CODE, chord, AF_POINTS)
    
    # Calculate position
    x_offset = abs(y) * math.tan(math.radians(sweep))
    z_offset = y * math.tan(math.radians(dihedral))
    
    x = ref_x + x_offset
    z = WING_Z + z_offset
    
    # Apply twist (washout) and position
    return transform_shape(wire, App.Vector(x, y, z), 0, -washout, 0)

def tail_section(y, chord_root, chord_tip, span, airfoil):
    """Create horizontal tail section."""
    t = abs(y) / span
    chord = chord_root + t * (chord_tip - chord_root)
    wire = make_airfoil_wire(airfoil, chord, 81)
    pos = App.Vector(TAIL_X, y, TAIL_Z)
    return transform_shape(wire, pos, 0, 0, 0)

# ----------------------------
# Build Model
# ----------------------------
doc = App.ActiveDocument

print("Building RC airplane model...")

# ----------------------------
# WING
# ----------------------------
print("Creating wing...")
# Create multiple sections for smooth loft
sections = []
y_positions = [0, SEMI_SPAN * 0.25, SEMI_SPAN * 0.5, SEMI_SPAN * 0.75, SEMI_SPAN]
washout_vals = [0, WASHOUT_TIP * 0.25, WASHOUT_TIP * 0.5, WASHOUT_TIP * 0.75, WASHOUT_TIP]

for y, wo in zip(y_positions, washout_vals):
    t = y / SEMI_SPAN
    chord = CHORD_ROOT + t * (CHORD_TIP - CHORD_ROOT)
    sec = wing_section(y, chord, SWEEP_LE, DIHEDRAL, wo, WING_X_AT_ROOT)
    sections.append(sec)

# Loft half wing
half_wing = Part.makeLoft(sections, True, False, False)

# Mirror for full wing
full_wing = half_wing.fuse(half_wing.mirror(App.Vector(0,0,0), App.Vector(0,1,0)))

wing_obj = doc.addObject("Part::Feature", "Wing")
wing_obj.Shape = full_wing
print("Wing complete.")

# ----------------------------
# FUSELAGE
# ----------------------------
print("Creating fuselage...")

# Fuselage cross-section (circular)
def fuse_profile(radius):
    # Create circular cross-section
    circle = Part.makeCircle(radius, App.Vector(0, 0, 0), App.Vector(1, 0, 0))
    return Part.Wire([circle])

# Fuselage sections
fuse_sections = []

# Nose (tapers to point)
for i, x in enumerate([0, NOSE_LEN * 0.5, NOSE_LEN]):
    scale = (i / 2.0) if i > 0 else 0.01
    r = FUSE_RADIUS * scale
    profile = fuse_profile(r)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

# Main body - constant circular cross-section
for x in [NOSE_LEN + 60, WING_X_AT_ROOT, WING_X_AT_ROOT + 120, TAIL_X - 60]:
    profile = fuse_profile(FUSE_RADIUS)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

# Tail cone - tapers to smaller circle
for i, x in enumerate([TAIL_X, TAIL_X + 50, FUSE_LEN]):
    scale = 1.0 - (i * 0.35)
    r = FUSE_RADIUS * scale
    profile = fuse_profile(r)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

fuselage = Part.makeLoft(fuse_sections, True, False, False)
fuse_obj = doc.addObject("Part::Feature", "Fuselage")
fuse_obj.Shape = fuselage
print("Fuselage complete.")

# ----------------------------
# HORIZONTAL TAIL
# ----------------------------
print("Creating horizontal stabilizer...")
htp_sections = []
y_positions_htp = [0, HTP_SEMI * 0.33, HTP_SEMI * 0.66, HTP_SEMI]
for y in y_positions_htp:
    sec = tail_section(y, HTP_CHORD_ROOT, HTP_CHORD_TIP, HTP_SEMI, HTP_AIRFOIL)
    htp_sections.append(sec)

half_htp = Part.makeLoft(htp_sections, True, False, False)
full_htp = half_htp.fuse(half_htp.mirror(App.Vector(TAIL_X, 0, TAIL_Z), App.Vector(0,1,0)))

htp_obj = doc.addObject("Part::Feature", "HorizontalStabilizer")
htp_obj.Shape = full_htp
print("Horizontal stabilizer complete.")

# ----------------------------
# VERTICAL TAIL
# ----------------------------
print("Creating vertical stabilizer...")
vtp_sections = []
z_positions = [0, VTP_HEIGHT * 0.33, VTP_HEIGHT * 0.66, VTP_HEIGHT]
for i, z in enumerate(z_positions):
    t = z / VTP_HEIGHT
    chord = VTP_CHORD_ROOT + t * (VTP_CHORD_TIP - VTP_CHORD_ROOT)
    wire = make_airfoil_wire(VTP_AIRFOIL, chord, 81)
    # Rotate 90 degrees around X-axis and position
    pos = App.Vector(TAIL_X, 0, TAIL_Z + z)
    wire = transform_shape(wire, pos, 0, 0, 90)
    vtp_sections.append(wire)

vtp = Part.makeLoft(vtp_sections, True, False, False)
vtp_obj = doc.addObject("Part::Feature", "VerticalStabilizer")
vtp_obj.Shape = vtp
print("Vertical stabilizer complete.")

# ----------------------------
# MOTOR MOUNT
# ----------------------------
print("Creating motor mount...")
motor_mount = Part.makeCylinder(MOTOR_DIAM/2 + 2, FIREWALL_THICKNESS, 
                                App.Vector(-FIREWALL_THICKNESS, 0, 0), 
                                App.Vector(1, 0, 0))
motor_obj = doc.addObject("Part::Feature", "MotorMount")
motor_obj.Shape = motor_mount
print("Motor mount complete.")

# ----------------------------
# COMPLETE ASSEMBLY
# ----------------------------
print("Assembling complete aircraft...")
complete_plane = fuselage.fuse(full_wing).fuse(full_htp).fuse(vtp).fuse(motor_mount)
plane_obj = doc.addObject("Part::Feature", "CompleteAircraft")
plane_obj.Shape = complete_plane

doc.recompute()
print("Model complete!")

# ----------------------------
# EXPORT
# ----------------------------
if EXPORT_STEP:
    try:
        import ImportGui
        ImportGui.export([plane_obj], EXPORT_PATH)
        print(f"Exported STEP file to: {EXPORT_PATH}")
    except Exception as e:
        print(f"Export failed: {e}")
        print("You can manually export via File > Export in FreeCAD")

print("\n" + "="*50)
print("RC AIRPLANE SPECIFICATIONS:")
print("="*50)
print(f"Wingspan: {SPAN} mm ({SPAN/25.4:.1f} inches)")
print(f"Wing Area: ~{(CHORD_ROOT + CHORD_TIP)/2 * SPAN / 100:.1f} cm²")
print(f"Fuselage Length: {FUSE_LEN} mm ({FUSE_LEN/25.4:.1f} inches)")
print(f"Airfoil: NACA {NACA_CODE}")
print(f"Dihedral: {DIHEDRAL}°")
print(f"Wing Sweep: {SWEEP_LE}°")
print(f"Washout: {WASHOUT_TIP}°")
print("="*50)
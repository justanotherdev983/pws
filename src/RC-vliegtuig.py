import math
import FreeCAD as App
import Part

# ----------------------------
# RC Vliegtuig 
# ----------------------------

DOC_NAME = "RC-vliegtuig_3D_ontwerp"
EXPORT_STEP = True
EXPORT_DIR = App.getUserAppDataDir()

# Aparte delen, zodat we het vliegtuig in een normale 20x20cm printer in delen geprint kan worden
EXPORT_PATHS = {
    'fuse_bottom': EXPORT_DIR + "01_fuselage_bottom.step",
    'fuse_top': EXPORT_DIR + "02_fuselage_top.step",
    'wing_left_inner': EXPORT_DIR + "03_wing_left_inner.step",
    'wing_left_outer': EXPORT_DIR + "04_wing_left_outer.step",
    'wing_right_inner': EXPORT_DIR + "05_wing_right_inner.step",
    'wing_right_outer': EXPORT_DIR + "06_wing_right_outer.step",
    'htp_left': EXPORT_DIR + "07_htail_left.step",
    'htp_right': EXPORT_DIR + "08_htail_right.step",
    'vtp': EXPORT_DIR + "09_vtail.step",
}

# ----------------------------
# Constanten 
# ----------------------------

# We gebruiken NACA 2412 als vleugelprofiel  
NACA_CODE = "2412"
SPAN = 750.0
SEMI_SPAN = SPAN * 0.5
CHORD_ROOT = 110.0
TAPER_RATIO = 0.62
CHORD_TIP = CHORD_ROOT * TAPER_RATIO
SWEEP_LE = 7.0
DIHEDRAL = 4.0
WASHOUT_TIP = 2.5
AF_POINTS = 121

# Fuselage
FUSE_LEN = 540.0
FUSE_RADIUS = 25.0
NOSE_LEN = 85.0
SHELL_THICKNESS = 2.0

# Horizontale stabilisator
HTP_SPAN = 260.0
HTP_SEMI = HTP_SPAN * 0.5
HTP_CHORD_ROOT = 62.0
HTP_CHORD_TIP = 44.0
HTP_AIRFOIL = "0012"

# Verticalw Stabilisator
VTP_HEIGHT = 78.0
VTP_CHORD_ROOT = 75.0
VTP_CHORD_TIP = 40.0
VTP_AIRFOIL = "0012"

# Ligging van de vleugel 
WING_X_AT_ROOT = 200.0
WING_Z = -12.0
TAIL_X = FUSE_LEN - 105.0
TAIL_Z = 0.0

# Split vliegtuig - door het midden van de romp, zodat we heb uitelkaar kunnen halen
SPLIT_Z = -FUSE_RADIUS * 0.5  # (iets onder centerline)

# Control Surfaces
AILERON_CHORD_RATIO = 0.25
AILERON_SPAN_START = 0.3
AILERON_SPAN_END = 0.9
ELEVATOR_CHORD_RATIO = 0.30
RUDDER_HEIGHT_RATIO = 0.40

# Servo dimensies (9g micro servo)
SERVO_LENGTH = 23.0
SERVO_WIDTH = 12.0
SERVO_HEIGHT = 29.0
SERVO_MOUNT_DEPTH = 20.0  

# Wing gesplit positie 
WING_SPLIT_Y = 180.0  # Split vleugel op 180mm (past op 200mm bed)

# ----------------------------
# List met NACA punten genereren
# ----------------------------
def naca4_points(code, n=121):
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0
    
    beta = [i * math.pi / (n - 1) for i in range(n)] # Beta verdeling, zodat er meer punten bij uiteindes zitten door cosinus
    x = [(1.0 - math.cos(b)) / 2.0 for b in beta] # Normaliseer beta x-coord tusen 0 en 1
    
    xu, zu, xl, zl = [], [], [], []
    
    for xi in x:
        yt = 5 * t * (0.2969 * math.sqrt(xi) - 0.1260 * xi - 0.3516 * xi**2 
                      + 0.2843 * xi**3 - 0.1015 * xi**4) # Definieer symmetrische voor voor NACA profiel
        
        # Camber lijn
        if xi < p and p > 0:
            yc = (m / p**2) * (2 * p * xi - xi**2)
            dyc_dx = (2 * m / p**2) * (p - xi)
        elif p > 0: # Andere parabolische kromming
            yc = (m / (1 - p)**2) * ((1 - 2 * p) + 2 * p * xi - xi**2)
            dyc_dx = (2 * m / (1 - p)**2) * (p - xi)
        else: # Geen camber bij p = 0
            yc = 0.0
            dyc_dx = 0.0
        
        theta = math.atan(dyc_dx)
        
        xu.append(xi - yt * math.sin(theta))
        zu.append(yc + yt * math.cos(theta))
        xl.append(xi + yt * math.sin(theta))
        zl.append(yc - yt * math.cos(theta))
    
    x_coords = xu + xl[::-1]
    z_coords = zu + zl[::-1]
    
    return list(zip(x_coords, z_coords))

def make_airfoil_wire(code, chord, n=121):
    points = naca4_points(code, n)
    pts = [App.Vector(x * chord, 0, z * chord) for x, z in points]
    
    spline = Part.BSplineCurve()
    spline.interpolate(pts, False)
    edge = spline.toShape()
    
    if edge.Vertexes[0].Point.distanceToPoint(edge.Vertexes[-1].Point) > 0.01:
        closing_edge = Part.LineSegment(edge.Vertexes[-1].Point, 
                                        edge.Vertexes[0].Point).toShape()
        wire = Part.Wire([edge, closing_edge])
    else:
        wire = Part.Wire([edge])
    
    return wire

def transform_shape(shape, pos, rot_x=0, rot_y=0, rot_z=0):
    s = shape.copy()
    rot = App.Rotation(App.Vector(1,0,0), rot_x)
    rot = rot.multiply(App.Rotation(App.Vector(0,1,0), rot_y))
    rot = rot.multiply(App.Rotation(App.Vector(0,0,1), rot_z))
    s.Placement = App.Placement(pos, rot)
    return s

def wing_section(y, chord, sweep, dihedral, washout, ref_x):
    wire = make_airfoil_wire(NACA_CODE, chord, AF_POINTS)
    
    x_offset = abs(y) * math.tan(math.radians(sweep))
    z_offset = y * math.tan(math.radians(dihedral))
    
    x = ref_x + x_offset
    z = WING_Z + z_offset
    
    return transform_shape(wire, App.Vector(x, y, z), 0, -washout, 0)

def tail_section(y, chord_root, chord_tip, span, airfoil):
    t = abs(y) / span
    chord = chord_root + t * (chord_tip - chord_root)
    wire = make_airfoil_wire(airfoil, chord, 81)
    pos = App.Vector(TAIL_X, y, TAIL_Z)
    return transform_shape(wire, pos, 0, 0, 0)

# ----------------------------
# Alle onderdelen bouwen 
# ----------------------------
doc = App.ActiveDocument


# Maak complete vleugel eerst

sections = []
y_positions = [0, SEMI_SPAN * 0.25, SEMI_SPAN * 0.5, SEMI_SPAN * 0.75, SEMI_SPAN]
washout_vals = [0, WASHOUT_TIP * 0.25, WASHOUT_TIP * 0.5, WASHOUT_TIP * 0.75, WASHOUT_TIP]

for y, wo in zip(y_positions, washout_vals):
    t = y / SEMI_SPAN
    chord = CHORD_ROOT + t * (CHORD_TIP - CHORD_ROOT)
    sec = wing_section(y, chord, SWEEP_LE, DIHEDRAL, wo, WING_X_AT_ROOT)
    sections.append(sec)

half_wing = Part.makeLoft(sections, True, False, False)
full_wing = half_wing.fuse(half_wing.mirror(App.Vector(0,0,0), App.Vector(0,1,0)))

# Servo bays toevoegen
aileron_servo_y = SEMI_SPAN * 0.6

t = aileron_servo_y / SEMI_SPAN
chord_at_servo = CHORD_ROOT + t * (CHORD_TIP - CHORD_ROOT)
x_offset_servo = aileron_servo_y * math.tan(math.radians(SWEEP_LE))
x_servo = WING_X_AT_ROOT + x_offset_servo + chord_at_servo * 0.5
z_offset_servo = aileron_servo_y * math.tan(math.radians(DIHEDRAL))
z_servo = WING_Z + z_offset_servo

servo_cavity_right = Part.makeBox(
    SERVO_LENGTH + 2, SERVO_WIDTH + 2, SERVO_MOUNT_DEPTH,
    App.Vector(x_servo - SERVO_LENGTH/2, aileron_servo_y - SERVO_WIDTH/2, z_servo - SERVO_MOUNT_DEPTH)
)

servo_cavity_left = Part.makeBox(
    SERVO_LENGTH + 2, SERVO_WIDTH + 2, SERVO_MOUNT_DEPTH,
    App.Vector(x_servo - SERVO_LENGTH/2, -aileron_servo_y - SERVO_WIDTH/2, z_servo - SERVO_MOUNT_DEPTH)
)

try:
    full_wing = full_wing.cut(servo_cavity_right).cut(servo_cavity_left)
except:
    print("Failed to create Servo bays")

print("Created wing")

# ----------------------------
# Vleugels splitsen voor printer 
# ----------------------------

# Maak split boxes voor vleugels
wing_split_box_inner = Part.makeBox(1000, WING_SPLIT_Y * 2, 1000,
                                     App.Vector(-500, -WING_SPLIT_Y, -500))

wing_split_box_outer_right = Part.makeBox(1000, 1000, 1000,
                                           App.Vector(-500, WING_SPLIT_Y, -500))

wing_split_box_outer_left = Part.makeBox(1000, 1000, 1000,
                                          App.Vector(-500, -WING_SPLIT_Y - 1000, -500))

try:
    # Inner deel (center + root)
    wing_inner = full_wing.common(wing_split_box_inner)
    
    # Outer delen
    wing_outer_right = full_wing.common(wing_split_box_outer_right)
    wing_outer_left = full_wing.common(wing_split_box_outer_left)
    
    # Voeg alignment pennen toe tussen inner en outer
    pen_diam = 3.0
    pen_height = 4.0
    
    # Pennen voor rechter vleugel
    pen_y_right = WING_SPLIT_Y
    t_pen = pen_y_right / SEMI_SPAN
    chord_pen = CHORD_ROOT + t_pen * (CHORD_TIP - CHORD_ROOT)
    x_pen_right = WING_X_AT_ROOT + pen_y_right * math.tan(math.radians(SWEEP_LE)) + chord_pen * 0.3
    z_pen_right = WING_Z + pen_y_right * math.tan(math.radians(DIHEDRAL))
    
    pen_locations_right = [
        App.Vector(x_pen_right, pen_y_right, z_pen_right),
        App.Vector(x_pen_right + chord_pen * 0.4, pen_y_right, z_pen_right),
    ]
    
    # Pennen toevoegen aan inner
    for loc in pen_locations_right:
        pen = Part.makeCylinder(pen_diam/2, pen_height, loc, App.Vector(0, 1, 0))
        wing_inner = wing_inner.fuse(pen)
    
    # Gaten in outer right
    for loc in pen_locations_right:
        hole = Part.makeCylinder(pen_diam/2 + 0.15, pen_height + 1, 
                                App.Vector(loc.x, loc.y - 0.5, loc.z), App.Vector(0, 1, 0))
        wing_outer_right = wing_outer_right.cut(hole)
    
    # Hetzelfde voor linker vleugel (gespiegeld)
    pen_y_left = -WING_SPLIT_Y
    x_pen_left = WING_X_AT_ROOT + WING_SPLIT_Y * math.tan(math.radians(SWEEP_LE)) + chord_pen * 0.3
    z_pen_left = WING_Z + WING_SPLIT_Y * math.tan(math.radians(DIHEDRAL))
    
    pen_locations_left = [
        App.Vector(x_pen_left, pen_y_left, z_pen_left),
        App.Vector(x_pen_left + chord_pen * 0.4, pen_y_left, z_pen_left),
    ]
    
    for loc in pen_locations_left:
        pen = Part.makeCylinder(pen_diam/2, pen_height, loc, App.Vector(0, -1, 0))
        wing_inner = wing_inner.fuse(pen)
    
    for loc in pen_locations_left:
        hole = Part.makeCylinder(pen_diam/2 + 0.15, pen_height + 1,
                                App.Vector(loc.x, loc.y + 0.5, loc.z), App.Vector(0, -1, 0))
        wing_outer_left = wing_outer_left.cut(hole)
    
    print("Split the wings successfully)")
    print("Added Alignment pennen")
    
except Exception as e:
    print(f"Failed to create split wings: {e}")
    wing_inner = full_wing
    wing_outer_right = None
    wing_outer_left = None

# ----------------------------
# FUSELAGE
# ----------------------------

def fuse_profile(radius):
    circle = Part.makeCircle(radius, App.Vector(0, 0, 0), App.Vector(1, 0, 0))
    return Part.Wire([circle])

fuse_sections = []

for i, x in enumerate([0, NOSE_LEN * 0.5, NOSE_LEN]):
    scale = (i / 2.0) if i > 0 else 0.01
    r = FUSE_RADIUS * scale
    profile = fuse_profile(r)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

for x in [NOSE_LEN + 50, WING_X_AT_ROOT, WING_X_AT_ROOT + 100, TAIL_X - 50]:
    profile = fuse_profile(FUSE_RADIUS)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

for i, x in enumerate([TAIL_X, TAIL_X + 45, FUSE_LEN]):
    scale = 1.0 - (i * 0.35)
    r = FUSE_RADIUS * scale
    profile = fuse_profile(r)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

fuselage = Part.makeLoft(fuse_sections, True, False, False)

try:
    fuselage_hollow = fuselage.makeThickness([fuselage.Faces[0]], -SHELL_THICKNESS, 1.e-3)
    print("Created hollow fuselagep")
except:
    print("Created solid fuselage")
    fuselage_hollow = fuselage

# Split fuselage
split_box_top = Part.makeBox(2000, 2000, 2000, App.Vector(-1000, -1000, SPLIT_Z))
split_box_bottom = Part.makeBox(2000, 2000, 2000, App.Vector(-1000, -1000, SPLIT_Z - 2000))

fuse_top = fuselage_hollow.common(split_box_top)
fuse_bottom = fuselage_hollow.common(split_box_bottom)

# Alignment pennen tussen fuse top/bottom
pin_locations_fuse = [
    App.Vector(150, 0, SPLIT_Z),
    App.Vector(320, 0, SPLIT_Z),
    App.Vector(460, 0, SPLIT_Z),
    App.Vector(250, 18, SPLIT_Z),
    App.Vector(250, -18, SPLIT_Z),
]

for loc in pin_locations_fuse:
    pen = Part.makeCylinder(1.5, 3, loc, App.Vector(0, 0, 1))
    fuse_bottom = fuse_bottom.fuse(pen)

for loc in pin_locations_fuse:
    hole = Part.makeCylinder(1.65, 4, App.Vector(loc.x, loc.y, loc.z - 0.5), App.Vector(0, 0, 1))
    fuse_top = fuse_top.cut(hole)

print("Created split fuselage")

# ----------------------------
# Horizontale staart 
# ----------------------------

htp_sections = []
y_positions_htp = [0, HTP_SEMI * 0.33, HTP_SEMI * 0.66, HTP_SEMI]
for y in y_positions_htp:
    sec = tail_section(y, HTP_CHORD_ROOT, HTP_CHORD_TIP, HTP_SEMI, HTP_AIRFOIL)
    htp_sections.append(sec)

half_htp = Part.makeLoft(htp_sections, True, False, False)
full_htp = half_htp.fuse(half_htp.mirror(App.Vector(TAIL_X, 0, TAIL_Z), App.Vector(0,1,0)))

# Servo bay elevator
elevator_servo_x = TAIL_X + HTP_CHORD_ROOT * 0.5
servo_bay_htp = Part.makeBox(
    SERVO_LENGTH + 2, SERVO_WIDTH + 2, SERVO_MOUNT_DEPTH,
    App.Vector(elevator_servo_x - SERVO_LENGTH/2, -SERVO_WIDTH/2, TAIL_Z - SERVO_MOUNT_DEPTH)
)

try:
    full_htp = full_htp.cut(servo_bay_htp)
    print("Created elevator servo bay")
except:
    print("Failed to create elevator servo bay")

# Split HTP in 2 delen
htp_split_box_left = Part.makeBox(1000, 1000, 1000, App.Vector(-500, -1000, -500))
htp_split_box_right = Part.makeBox(1000, 1000, 1000, App.Vector(-500, 0, -500))

htp_left = full_htp.common(htp_split_box_left)
htp_right = full_htp.common(htp_split_box_right)

# Alignment pennen
pen_htp_loc = [
    App.Vector(TAIL_X + 30, 0, TAIL_Z),
    App.Vector(TAIL_X + 50, 0, TAIL_Z),
]

for loc in pen_htp_loc:
    pen = Part.makeCylinder(1.5, 3, loc, App.Vector(0, 1, 0))
    htp_left = htp_left.fuse(pen)

for loc in pen_htp_loc:
    hole = Part.makeCylinder(1.65, 4, App.Vector(loc.x, loc.y - 0.5, loc.z), App.Vector(0, 1, 0))
    htp_right = htp_right.cut(hole)

print("    ✓ HTP gesplitst (left/right)")

# ----------------------------
# Verticale staart
# ----------------------------
vtp_sections = []
z_positions = [0, VTP_HEIGHT * 0.33, VTP_HEIGHT * 0.66, VTP_HEIGHT]
for i, z in enumerate(z_positions):
    t = z / VTP_HEIGHT
    chord = VTP_CHORD_ROOT + t * (VTP_CHORD_TIP - VTP_CHORD_ROOT)
    wire = make_airfoil_wire(VTP_AIRFOIL, chord, 81)
    pos = App.Vector(TAIL_X, 0, TAIL_Z + z)
    wire = transform_shape(wire, pos, 90, 0, 0)
    vtp_sections.append(wire)

vtp = Part.makeLoft(vtp_sections, True, False, False)

# Servo bay rudder
rudder_servo_z = VTP_HEIGHT * 0.3
rudder_servo_x = TAIL_X + VTP_CHORD_ROOT * 0.4

servo_bay_vtp = Part.makeBox(
    SERVO_LENGTH + 2, SERVO_WIDTH + 2, SERVO_HEIGHT + 2,
    App.Vector(rudder_servo_x - SERVO_LENGTH/2, -SERVO_WIDTH/2 - SERVO_MOUNT_DEPTH, rudder_servo_z - SERVO_HEIGHT/2)
)

try:
    vtp = vtp.cut(servo_bay_vtp)
    print("Created rudder servo bay")
except:
    print("Failed to create rudder servo bay")

print("    ✓ VTP compleet")

# ----------------------------
# Gesplitte onderdelen in eigen document
# ----------------------------
print("\nAdding objects to different documents...")

# Fuselage
fuse_bottom_obj = doc.addObject("Part::Feature", "01_Fuselage_Bottom")
fuse_bottom_obj.Shape = fuse_bottom

fuse_top_obj = doc.addObject("Part::Feature", "02_Fuselage_Top")
fuse_top_obj.Shape = fuse_top

# Wings
wing_inner_obj = doc.addObject("Part::Feature", "03_Wing_Center")
wing_inner_obj.Shape = wing_inner

if wing_outer_left:
    wing_left_obj = doc.addObject("Part::Feature", "04_Wing_Left_Outer")
    wing_left_obj.Shape = wing_outer_left

if wing_outer_right:
    wing_right_obj = doc.addObject("Part::Feature", "05_Wing_Right_Outer")
    wing_right_obj.Shape = wing_outer_right

# Tails
htp_left_obj = doc.addObject("Part::Feature", "06_HTail_Left")
htp_left_obj.Shape = htp_left

htp_right_obj = doc.addObject("Part::Feature", "07_HTail_Right")
htp_right_obj.Shape = htp_right

vtp_obj = doc.addObject("Part::Feature", "08_VTail")
vtp_obj.Shape = vtp

doc.recompute()

# ----------------------------
# EXPORT
# ----------------------------
print("\nExporting")
if EXPORT_STEP:
    try:
        import ImportGui
        
        parts_to_export = [
            (fuse_bottom_obj, 'fuse_bottom'),
            (fuse_top_obj, 'fuse_top'),
            (wing_inner_obj, 'wing_left_inner'),  # center = inner
            (wing_left_obj if wing_outer_left else None, 'wing_left_outer'),
            (wing_right_obj if wing_outer_right else None, 'wing_right_inner'),  # used for outer right
            (htp_left_obj, 'htp_left'),
            (htp_right_obj, 'htp_right'),
            (vtp_obj, 'vtp'),
        ]
        
        exported_count = 0
        for obj, key in parts_to_export:
            if obj and key in EXPORT_PATHS:
                ImportGui.export([obj], EXPORT_PATHS[key])
                print(f"    ✓ {EXPORT_PATHS[key]}")
                exported_count += 1
        
        print("Exported successfully!")
        
    except Exception as e:
        print(f"Failed to export: {e}")

print("Created plane successfully")

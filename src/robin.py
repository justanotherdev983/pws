import math
import FreeCAD as App
import Part

# ----------------------------
# RC Airplane Parameters (75cm wingspan versie)
# ----------------------------
DOC_NAME = "RC_Plane_75cm"
EXPORT_STEP = True
EXPORT_PATH = App.getUserAppDataDir() + "rc_plane_75cm.step"

# Wing - NACA 2412 (2% camber, 40% max camber, 12% thickness)
NACA_CODE = "2412"
SPAN = 750.0              # volledige spanwijdte, mm (75 cm)
SEMI_SPAN = SPAN * 0.5
CHORD_ROOT = 110.0        # mm (aangepast voor oppervlak ~0,13 m²)
TAPER_RATIO = 0.62        # tip/root ratio
CHORD_TIP = CHORD_ROOT * TAPER_RATIO
SWEEP_LE = 7.0            # graden
DIHEDRAL = 4.0            # graden (V-vorm)
WASHOUT_TIP = 2.5         # graden (vermindert tip stall)
AF_POINTS = 121           # airfoil resolutie

# Fuselage
FUSE_LEN = 540.0          # mm (54 cm - midden van gewenst bereik)
FUSE_RADIUS = 25.0        # mm (2.5 cm radius - compact)
NOSE_LEN = 85.0           # mm

# Horizontal Stabilizer (schaalbaar met vliegtuig)
HTP_SPAN = 260.0          # mm
HTP_SEMI = HTP_SPAN * 0.5
HTP_CHORD_ROOT = 62.0
HTP_CHORD_TIP = 44.0
HTP_AIRFOIL = "0012"      # symmetrisch

# Vertical Stabilizer
VTP_HEIGHT = 78.0
VTP_CHORD_ROOT = 75.0
VTP_CHORD_TIP = 40.0
VTP_AIRFOIL = "0012"

# Motor Mount (voor brushless motor)
MOTOR_DIAM = 28.0         # mm (typisch ~28mm outrunner motor)
FIREWALL_THICKNESS = 2.5  # mm (steviger voor brushless)

# Placement
WING_X_AT_ROOT = 200.0    # vleugel LE vanaf neus
WING_Z = -12.0            # licht onder fuselage centerline
TAIL_X = FUSE_LEN - 105.0
TAIL_Z = 0.0

# ----------------------------
# NACA 4-digit Airfoil Generator
# ----------------------------
def naca4_points(code, n=121):
    """
    Genereer NACA 4-cijferige airfoil coördinaten.
    Retourneert lijst van (x, z) tuples die gesloten lus vormen.
    code: string zoals "2412" (m, p, t)
    """
    m = int(code[0]) / 100.0      # max camber
    p = int(code[1]) / 10.0       # positie van max camber
    t = int(code[2:]) / 100.0     # dikte
    
    # Cosinus spacing voor betere leading edge
    beta = [i * math.pi / (n - 1) for i in range(n)]
    x = [(1.0 - math.cos(b)) / 2.0 for b in beta]
    
    xu, zu, xl, zl = [], [], [], []
    
    for xi in x:
        # Dikte distributie
        yt = 5 * t * (0.2969 * math.sqrt(xi) - 0.1260 * xi - 0.3516 * xi**2 
                      + 0.2843 * xi**3 - 0.1015 * xi**4)
        
        # Gemiddelde camber lijn
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
    
    # Creëer gesloten lus: bovenvlak + ondervlak omgekeerd
    x_coords = xu + xl[::-1]
    z_coords = zu + zl[::-1]
    
    return list(zip(x_coords, z_coords))

# ----------------------------
# Helper Functions
# ----------------------------
def make_airfoil_wire(code, chord, n=121):
    """Creëer gesloten wire van NACA airfoil, geschaald naar chord lengte."""
    points = naca4_points(code, n)
    pts = [App.Vector(x * chord, 0, z * chord) for x, z in points]
    
    # Creëer B-spline door punten
    spline = Part.BSplineCurve()
    spline.interpolate(pts, False)
    edge = spline.toShape()
    
    # Sluit de wire
    if edge.Vertexes[0].Point.distanceToPoint(edge.Vertexes[-1].Point) > 0.01:
        closing_edge = Part.LineSegment(edge.Vertexes[-1].Point, 
                                        edge.Vertexes[0].Point).toShape()
        wire = Part.Wire([edge, closing_edge])
    else:
        wire = Part.Wire([edge])
    
    return wire

def transform_shape(shape, pos, rot_x=0, rot_y=0, rot_z=0):
    """Transformeer shape: roteer en verplaats."""
    s = shape.copy()
    rot = App.Rotation(App.Vector(1,0,0), rot_x)
    rot = rot.multiply(App.Rotation(App.Vector(0,1,0), rot_y))
    rot = rot.multiply(App.Rotation(App.Vector(0,0,1), rot_z))
    s.Placement = App.Placement(pos, rot)
    return s

def wing_section(y, chord, sweep, dihedral, washout, ref_x):
    """Creëer vleugelgedeelte op spanwise positie y."""
    wire = make_airfoil_wire(NACA_CODE, chord, AF_POINTS)
    
    # Bereken positie
    x_offset = abs(y) * math.tan(math.radians(sweep))
    z_offset = y * math.tan(math.radians(dihedral))
    
    x = ref_x + x_offset
    z = WING_Z + z_offset
    
    # Pas twist (washout) en positie toe
    return transform_shape(wire, App.Vector(x, y, z), 0, -washout, 0)

def tail_section(y, chord_root, chord_tip, span, airfoil):
    """Creëer horizontale staart sectie."""
    t = abs(y) / span
    chord = chord_root + t * (chord_tip - chord_root)
    wire = make_airfoil_wire(airfoil, chord, 81)
    pos = App.Vector(TAIL_X, y, TAIL_Z)
    return transform_shape(wire, pos, 0, 0, 0)

# ----------------------------
# Build Model
# ----------------------------
doc = App.ActiveDocument

print("Bouwen RC vliegtuig model (75cm span)...")

# ----------------------------
# WING
# ----------------------------
print("Vleugel maken...")
# Creëer meerdere secties voor vloeiende loft
sections = []
y_positions = [0, SEMI_SPAN * 0.25, SEMI_SPAN * 0.5, SEMI_SPAN * 0.75, SEMI_SPAN]
washout_vals = [0, WASHOUT_TIP * 0.25, WASHOUT_TIP * 0.5, WASHOUT_TIP * 0.75, WASHOUT_TIP]

for y, wo in zip(y_positions, washout_vals):
    t = y / SEMI_SPAN
    chord = CHORD_ROOT + t * (CHORD_TIP - CHORD_ROOT)
    sec = wing_section(y, chord, SWEEP_LE, DIHEDRAL, wo, WING_X_AT_ROOT)
    sections.append(sec)

# Loft halve vleugel
half_wing = Part.makeLoft(sections, True, False, False)

# Spiegel voor volledige vleugel
full_wing = half_wing.fuse(half_wing.mirror(App.Vector(0,0,0), App.Vector(0,1,0)))

wing_obj = doc.addObject("Part::Feature", "Wing")
wing_obj.Shape = full_wing
print("Vleugel compleet.")

# ----------------------------
# FUSELAGE
# ----------------------------
print("Romp maken...")

# Fuselage cross-section (rond)
def fuse_profile(radius):
    circle = Part.makeCircle(radius, App.Vector(0, 0, 0), App.Vector(1, 0, 0))
    return Part.Wire([circle])

# Fuselage secties
fuse_sections = []

# Neus (loopt taps toe naar punt)
for i, x in enumerate([0, NOSE_LEN * 0.5, NOSE_LEN]):
    scale = (i / 2.0) if i > 0 else 0.01
    r = FUSE_RADIUS * scale
    profile = fuse_profile(r)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

# Hoofdgedeelte - constante ronde doorsnede
for x in [NOSE_LEN + 50, WING_X_AT_ROOT, WING_X_AT_ROOT + 100, TAIL_X - 50]:
    profile = fuse_profile(FUSE_RADIUS)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

# Staartconus - loopt taps toe naar kleinere cirkel
for i, x in enumerate([TAIL_X, TAIL_X + 45, FUSE_LEN]):
    scale = 1.0 - (i * 0.35)
    r = FUSE_RADIUS * scale
    profile = fuse_profile(r)
    profile.Placement = App.Placement(App.Vector(x, 0, 0), App.Rotation())
    fuse_sections.append(profile)

fuselage = Part.makeLoft(fuse_sections, True, False, False)
fuse_obj = doc.addObject("Part::Feature", "Fuselage")
fuse_obj.Shape = fuselage
print("Romp compleet.")

# ----------------------------
# HORIZONTAL TAIL
# ----------------------------
print("Horizontale stabilisator maken...")
htp_sections = []
y_positions_htp = [0, HTP_SEMI * 0.33, HTP_SEMI * 0.66, HTP_SEMI]
for y in y_positions_htp:
    sec = tail_section(y, HTP_CHORD_ROOT, HTP_CHORD_TIP, HTP_SEMI, HTP_AIRFOIL)
    htp_sections.append(sec)

half_htp = Part.makeLoft(htp_sections, True, False, False)
full_htp = half_htp.fuse(half_htp.mirror(App.Vector(TAIL_X, 0, TAIL_Z), App.Vector(0,1,0)))

htp_obj = doc.addObject("Part::Feature", "HorizontalStabilizer")
htp_obj.Shape = full_htp
print("Horizontale stabilisator compleet.")

# ----------------------------
# VERTICAL TAIL
# ----------------------------
print("Verticale stabilisator maken...")
vtp_sections = []
z_positions = [0, VTP_HEIGHT * 0.33, VTP_HEIGHT * 0.66, VTP_HEIGHT]
for i, z in enumerate(z_positions):
    t = z / VTP_HEIGHT
    chord = VTP_CHORD_ROOT + t * (VTP_CHORD_TIP - VTP_CHORD_ROOT)
    wire = make_airfoil_wire(VTP_AIRFOIL, chord, 81)
    # Positioneer op top van fuselage - start bij TAIL_Z (bovenkant romp)
    pos = App.Vector(TAIL_X, 0, TAIL_Z + z)
    # Roteer 90 graden om X-as zodat airfoil verticaal staat
    wire = transform_shape(wire, pos, 90, 0, 0)
    vtp_sections.append(wire)

vtp = Part.makeLoft(vtp_sections, True, False, False)
vtp_obj = doc.addObject("Part::Feature", "VerticalStabilizer")
vtp_obj.Shape = vtp
print("Verticale stabilisator compleet.")

# ----------------------------
# MOTOR MOUNT
# ----------------------------
print("Motor bevestiging maken...")
motor_mount = Part.makeCylinder(MOTOR_DIAM/2 + 2.5, FIREWALL_THICKNESS, 
                                App.Vector(-FIREWALL_THICKNESS, 0, 0), 
                                App.Vector(1, 0, 0))
motor_obj = doc.addObject("Part::Feature", "MotorMount")
motor_obj.Shape = motor_mount
print("Motor bevestiging compleet.")

# ----------------------------
# COMPLETE ASSEMBLY
# ----------------------------
print("Vliegtuig assembleren...")
complete_plane = fuselage.fuse(full_wing).fuse(full_htp).fuse(vtp).fuse(motor_mount)
plane_obj = doc.addObject("Part::Feature", "CompleteAircraft")
plane_obj.Shape = complete_plane

doc.recompute()
print("Model compleet!")

# ----------------------------
# EXPORT
# ----------------------------
if EXPORT_STEP:
    try:
        import ImportGui
        ImportGui.export([plane_obj], EXPORT_PATH)
        print(f"STEP bestand geëxporteerd naar: {EXPORT_PATH}")
    except Exception as e:
        print(f"Export mislukt: {e}")
        print("Je kunt handmatig exporteren via File > Export in FreeCAD")

# ----------------------------
# SPECIFICATIONS
# ----------------------------
wing_area_cm2 = (CHORD_ROOT + CHORD_TIP) / 2 * SPAN / 100
wing_area_m2 = wing_area_cm2 / 10000

print("\n" + "="*60)
print("RC VLIEGTUIG SPECIFICATIES:")
print("="*60)
print(f"Spanwijdte: {SPAN} mm ({SPAN/10:.1f} cm)")
print(f"Vleugeloppervlak: {wing_area_cm2:.1f} cm² ({wing_area_m2:.3f} m²)")
print(f"Romp lengte: {FUSE_LEN} mm ({FUSE_LEN/10:.1f} cm)")
print(f"Airfoil: NACA {NACA_CODE}")
print(f"Dihedral: {DIHEDRAL}°")
print(f"Vleugel Sweep: {SWEEP_LE}°")
print(f"Washout: {WASHOUT_TIP}°")
print(f"\nGemiddelde chord: {(CHORD_ROOT + CHORD_TIP)/2:.1f} mm")
print(f"Taper ratio: {TAPER_RATIO:.2f}")
print("="*60)
print("\nGESCHIKT VOOR:")
print("- 3S LiPo 2200mAh")
print("- Brushless motor (~1000-1400 KV)")
print("- 30A ESC met BEC")
print("- 9x4.5 of 10x4.5 propeller")
print("- 4x 9g micro servo's")
print("="*60)

# Towfish RevC — geometry, mass and buoyancy summary

Source model: `towfishRevC.step` (98 bodies). Geometry was measured from the STEP's exact B-rep volumes, centroids and bounds, plus mesh slices of the wings. Masses and the centre of gravity come from the owner's weighings. Units are mm, g and cm³ unless stated otherwise.

Status flags used below: **[measured]** taken from the CAD model; **[owner]** given by the owner from the physical fish; **[estimate]** computed from assumptions stated next to it.

---

## 1. Coordinate frame (STEP / model frame)

| Axis | Direction | Notes |
|---|---|---|
| **Y** | Tow axis. **−Y = forward** (dome/nose), **+Y = aft** (tail) | Tube axis lies on x = 0, z = 0 |
| **X** | Vertical. **+X assumed up** (the bar is on the +X side) | Up-direction **not confirmed** by owner |
| **Z** | Spanwise (wings at ±Z) | Port/starboard sign not confirmed |

- Origin: on the tube axis, **182.0 mm forward of the outer (back) face of the aft end cap**.
- The aft end-cap back face is at y = +182.0, with its centre at (0, 182, 0).
- To convert to "distance forward of the end-cap back face", use `d = 182.0 − y`.

---

## 2. Mass and buoyancy

### 2.1 Mass
| Item | Value | Status |
|---|---|---|
| **Total weight in air** | **7.1 lb = 3220.5 g** | [owner] Includes internal wiring and external connectors/cables that are *not* in the model. An earlier figure of 7.54 lb was with a different pair of wings installed; disregard it. |
| Wing, each | 156.57 g | [owner] PETG, gyroid floodable infill |
| Tail fin | 44.4 g | [estimate] Same PETG print as the wings, scaled by volume: 198.22 cm³ × (156.57 g / 699.55 cm³) |
| Aluminium U-channel bar | 197.2 g | [estimate] 73.05 cm³ × 2.70 g/cm³ (6061) |
| Everything else (pressure housing + contents, clamps, servos, wiring) | 2665.8 g | [estimate] Remainder of the total |

### 2.2 Displaced volume (model) [estimate]
These assumptions match the physical fish as described by the owner:
- The pressure housing is sealed, and its 5 open Ø10 cable holes in the end cap are treated as plugged flush.
- **The wings flood** (holes plus gyroid infill), so only their plastic displaces. That is 156.57 g ÷ 1.27 g/cm³ = 123.3 cm³ each.
- **The tail fin does not flood**, so its full envelope of 198.22 cm³ displaces.
- The servos are modelled as solid boxes of 67.5 cm³ each (45 mm × 2 measured dimensions).

| Component | Displaced volume (cm³) | Volume centroid (x, y, z) |
|---|---|---|
| Pressure-housing envelope (dome, tube, flanges, end cap) | 2411.78 | (0, 59.8, −0.1) |
| Wings, flooded (plastic only) | 2 × 123.3 = 246.6 | (−9.2, 67.2, ±165.8) |
| Tail fin (sealed) | 198.22 | (30.6, 630.2, 0) |
| Tube clamp | 148.66 | (−3.0, 30.0, 0.1) |
| Bar clamp | 9.70 | (77.3, 30.0, 0.9) |
| Aluminium U-channel bar | 73.05 | (70.1, 130.0, 0) |
| Servos (2 × box) | 135.0 | about (−9.75, 40, ±107.7) |
| Countersunk plates, cords, handles | 6.82 | — |
| Overlap correction | −4.33 | — |
| **Total** | **3225.5** | **CB = (2.44, 94.7, −0.08)** |

### 2.3 Net buoyancy (model volume vs. the 3220.5 g weight)
| Water | Density | Buoyancy | **Net** |
|---|---|---|---|
| Fresh | 0.9982 g/cm³ | 3219.7 g | **−0.8 g (neutral)** |
| Sea | 1.025 g/cm³ | 3306.1 g | **+85.6 g (buoyant)** |

The owner reports the fish as **slightly positively buoyant**. That fits the model being neutral in fresh water once the unmodelled displacement of the external connectors, penetrators and cables is added.

### 2.4 Centre of buoyancy vs. centre of gravity
| Point | y (mm) | Forward of end-cap back face | Other | Status |
|---|---|---|---|---|
| **CB** | **94.7** | 87.3 mm | x = +2.4 (above the tube axis if +X is up), z ≈ 0 | [estimate] |
| **CG** | **105.8** | **76.2 mm (3.00 in)** | x and z unknown (not measured) | [owner] balanced in air, fore/aft only |
| **Tow / mount point** | **−326.0** | **508 mm (20.0 in)** | Top of the "top rod" that carries the fish; its height above the bar is unknown | [owner] |

- **CG is 11.1 mm aft of CB.** In still water with W ≈ B, that gives a pitch moment of about 0.35 N·m nose-up, assuming CG and CB are at the same height.
- The **vertical CG is unknown**, so pitch stiffness can't be computed yet. A hang test would supply it: hang the fish from the tow point and record the trim angle.

### 2.5 Drag (first-cut estimate) [estimate]
| Speed | Total drag |
|---|---|
| 0.6 m/s | about 2.1 N |
| 1.0 m/s | about 5.9 N |

- The tube clamp (about 31 cm² exposed frontal area) and the blunt tail-fin edges are major contributors.
- Wing induced drag is about L²/107 N at 0.6 m/s and about L²/297 N at 1.0 m/s, with L in N.

---

## 3. Hull (pressure housing) dimensions [measured]

All parts are coaxial on the Y axis (x = 0, z = 0).

| Part | y from → to | Length | Diameter / size | Volume (B-rep) |
|---|---|---|---|---|
| Front dome (acrylic, WTE4-P-DOME-R4) | −85.3 → −36.0 | 49.3 | OD 104.6 | 56.29 cm³ (material) |
| Dome retaining ring | −42.5 → −36.0 | 6.5 | OD about 118.4 | 13.1 |
| Front locking flange | −36.0 → 89.0 (body); seal face near y = −30 | — | OD about 114.3 | 45.68 |
| **Main tube** (WTE4-M-LOCKING-TUBE-200) | **−30.0 → +170.0** | **200.0** | **OD 112.0, ID 104.16** | 261.87 |
| Aft locking flange | 51.0 → 176.0 (body) | — | OD about 114.3 | 45.68 |
| **Aft end cap** (WTE4-END-CAP-5XM10) | **172.0 → 182.0** | 10.0 | OD about 120 | 82.68 |

- **Overall pressure-housing length** (dome tip to end-cap back face): **267.3 mm** (y −85.3 → +182.0).
- **Sealed envelope** (outer surfaces, end-cap holes plugged): **2411.78 cm³**, centroid y = 59.8.
- **End-cap holes:** five Ø10.2 through holes at (x, z) = (±26.63, −10.91), (±16.46, +20.40) and (0, −30.25). Cables and connectors fit through these but are **not modelled**.
- **Tube clamp:** x −75.0 → +70.3, y 9.0 → 51.0 (42 mm wide), z ±75.0; 148.66 cm³.
- **Bar clamp:** x 70.3 → 83.3, y 9.0 → 51.0, z −14.4 → +15.6; 9.70 cm³.

### Bar and tail
| Part | Extent | Notes |
|---|---|---|
| **Aluminium U-channel bar** | y −479.6 → +739.6 (**1219.2 mm = 48 in**); x 65.3 → 78.1 (12.8 mm); z ±6.6 (13.2 mm) | Runs along the top (+X) of the fish. 73.05 cm³. |
| **Tail fin** | y 584.8 → 674.8 (**90 mm chord**); x −45.0 → +78.2; z ±107.3 (**214.6 mm span**) | Sealed, not flooded. 198.22 cm³. Centroid (30.6, 630.2, 0). Blunt edges. |

---

## 4. Wings [measured]

There are two wings, **mirror images about z = 0**: `bigliftwing_towfish` at +Z and `secondbigliftwing_towfish` at −Z. They are PETG prints with floodable gyroid infill, and each is servo-actuated (a BLUESERVO is embedded in each wing).

### 4.1 Planform
| Quantity | Value |
|---|---|
| Span per wing (root → tip) | **175.3 mm** (\|z\| 76.2 → 251.5) |
| Tip-to-tip span | **503.0 mm** |
| Root station | \|z\| = 76.2, which is 20.2 mm outboard of the tube's outer surface (tube radius 56) |
| **Root chord** | **200.1 mm**, constant from \|z\| 76.2 to about 150 |
| Taper | Linear from \|z\| ≈ 150 to the tip |
| **Tip chord** | **≈ 165.5 mm** at \|z\| 250 (taper ratio about 0.83) |
| Planform area per wing | ≈ **333 cm²** from slices (an earlier bounding estimate said about 380 cm²; use 333) |
| Total wing planform area | ≈ 666 cm² |
| Leading edge at root | y = −20.6 (9.4 mm aft of the tube's front end at y −30) |
| Trailing edge at root | y = +179.5 (just aft of the tube end at y 170; ahead of the end cap at 172–182) |
| Leading-edge sweep | 0° inboard; the LE moves aft outboard (y −20.6 → −2.2 at the tip) |
| Trailing-edge sweep | 0° inboard; the TE moves forward outboard (y 179.5 → 163.3) |
| Root quarter-chord | y ≈ +29.4 |
| Mean aerodynamic chord | ≈ 189 mm (from slices) |
| Wing volume centroid (solid envelope) | (−9.2, ±67.2, ±165.8): read as x = −9.2, y = 67.2, \|z\| = 165.8 |

**Span stations** (each wing; x is the chord-line height, +X assumed up):

| \|z\| | LE y | TE y | Chord | Max thickness | t/c | Chord-line incidence |
|---|---|---|---|---|---|---|
| 76.2–130 | −20.6 | 179.5 | 200.1 | 37.1 | 18.5% | 0.40° |
| 160 | −19.1 | 178.0 | 197.0 | 36.5 | 18.5% | 0.42° |
| 190 | −13.7 | 172.7 | 186.4 | 34.5 | 18.5% | 0.26° |
| 220 | −8.5 | 167.4 | 175.8 | 32.7 | 18.6% | 0.32° |
| 245 | −4.1 | 163.0 | 167.0 | 31.2 | 18.7% | 0.38° |

- **Section:** about **18.5% thick**, constant across the span. The airfoil family isn't specified in the STEP.
- **Vertical position:** the chord line sits at x ≈ −4.1 (LE) to −2.7 (TE). That is about 3–4 mm on the −X side of the tube axis; with +X up, slightly below the axis.
- **Incidence:** about 0.4° as modelled (the LE is lower than the TE if +X is up). The servo rotates this, and the commanded range isn't known.
- **Tip endplate:** at \|z\| ≈ 250–251.5 each wing has a downturned tip feature reaching x −62.0 (about 58 mm below the chord line) and aft to y 222.3. This is what sets the wing bounding box: x −62 → +8.5, y −20.6 → +222.3.
- **Servos:** the embedded servo bodies sit at about x −22…+3, y 17.5…62.5, \|z\| 77.7…137.7.

### 4.2 Wing mass and buoyancy
| | Value |
|---|---|
| Mass, each | 156.57 g [owner] |
| Envelope volume, each | 699.55 cm³ [measured] |
| Displaced volume when flooded, each | 123.3 cm³ (PETG at 1.27 g/cm³) [estimate] |
| Net in fresh water, each | about −33.5 g, i.e. sinks (156.57 − 123.3 × 0.9982) |

---

## 5. Not in the model / open items
- External connectors, penetrators and cables on the aft end cap, plus internal wiring. These are included in the 3220.5 g weight but not in the 3225.5 cm³ volume.
- Internal component masses and positions, so the **CG height and lateral position are unknown**.
- Confirmation of the up-axis (+X assumed) and of the tow-rod geometry: its height above the bar, and whether it is rigid or pivoting.
- The wing servo deflection range.
- Separately, a redesigned replacement wing exists in this workspace: NACA 4415, root chord 222 mm, taper 0.75, 196.68 mm span per wing (1.6 in less tip-to-tip than the reference wing). Files: `towfish_wing_naca4415.step` and `.solidcraft`. The numbers above describe the wings **in `towfishRevC.step`**, not this redesign.

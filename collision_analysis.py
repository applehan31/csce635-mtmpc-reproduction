"""Read-only collision geometry analysis for the 5 failed mt-MPC trials."""
import math

OBSTACLES = [
    (1.0, 1.0, 0.3, 'obs0'),
    (2.0, 1.5, 0.3, 'obs1'),
    (1.5, 2.5, 0.3, 'obs2'),
    (0.8, 2.0, 0.2, 'obs3'),
    (2.3, 2.3, 0.4, 'obs4'),
    (1.2, 0.5, 0.25,'obs5'),
]
CRASH_TOL = 0.05

def surf_dist(px, py, cx, cy, r):
    return math.sqrt((px-cx)**2 + (py-cy)**2) - r

def nearest_obs(px, py):
    best, best_d = None, 1e9
    for cx, cy, r, name in OBSTACLES:
        d = surf_dist(px, py, cx, cy, r)
        if d < best_d:
            best_d = d
            best = (cx, cy, r, name)
    return best, best_d

# Last two logged positions before collision, to estimate v_safe direction
CASES = [
    # label,          p_prev,          p_curr,          la_used
    ('C1 v0.5 la0.40 t0', (1.055,1.991), (1.052,1.987), 0.40),
    ('C2 v0.5 la0.60 t0', (1.143,2.435), (1.152,2.462), 0.60),
    ('C3 v0.5 la0.60 t1', (2.007,1.954), (2.008,1.958), 0.60),
    ('C4 v0.5 la0.60 t2', (1.600,1.450), (1.635,1.443), 0.60),
    ('C5 v1.0 la0.60 t1', (1.472,2.110), (1.482,2.133), 0.60),
]

print("=" * 100)
print("COLLISION GEOMETRY ANALYSIS — 5 failed mt-MPC trials")
print("  v_safe direction estimated from last two logged positions before collision")
print("  Carrot = pos + lookahead * normalize(v_safe)")
print("  'Inside obstacle' = carrot→obs surface < 0  (PID target inside physical body)")
print("=" * 100)

for label, p_prev, p_curr, la in CASES:
    px, py = p_curr
    vx = px - p_prev[0]
    vy = py - p_prev[1]
    vmag = math.sqrt(vx**2 + vy**2)

    (cx, cy, r, oname), d_surf = nearest_obs(px, py)

    print(f"\n{label}")
    print(f"  Collision pos    : ({px:.4f}, {py:.4f})")
    print(f"  Nearest obstacle : {oname} center=({cx},{cy}) r={r}  surface_dist={d_surf:.4f}m")

    if vmag > 1e-9:
        nx, ny = vx/vmag, vy/vmag
        print(f"  v_safe direction : ({nx:+.4f}, {ny:+.4f})  |v_step|={vmag:.5f} m/step")

        cx_c = px + la * nx
        cy_c = py + la * ny
        cd = math.sqrt((cx_c - cx)**2 + (cy_c - cy)**2) - r
        inside = "*** CARROT INSIDE OBSTACLE ***" if cd < 0 else "carrot outside"
        print(f"  Carrot (la={la:.2f}) : ({cx_c:.4f}, {cy_c:.4f})  carrot→obs_surface={cd:+.4f}m  [{inside}]")

        print(f"  Lookahead sweep (same direction):")
        for la_t in [0.00, 0.10, 0.20, 0.30, 0.40, 0.60]:
            if la_t == 0.0:
                d_c = d_surf
                c_str = f"({px:.3f},{py:.3f})"
            else:
                cx_t = px + la_t * nx
                cy_t = py + la_t * ny
                d_c = math.sqrt((cx_t - cx)**2 + (cy_t - cy)**2) - r
                c_str = f"({cx_t:.3f},{cy_t:.3f})"
            flag = " *** INSIDE" if d_c < 0 else (" <-- threshold" if abs(d_c) < 0.05 else "")
            print(f"    la={la_t:.2f}  carrot={c_str}  surface={d_c:+.4f}m{flag}")
    else:
        print(f"  v_safe           : ~zero (hover)  — tracking error caused drift")

print("\n" + "=" * 100)
print("SUMMARY")
print("=" * 100)
for label, p_prev, p_curr, la in CASES:
    px, py = p_curr
    vx = px - p_prev[0]; vy = py - p_prev[1]
    vmag = math.sqrt(vx**2 + vy**2)
    (cx, cy, r, oname), d_surf = nearest_obs(px, py)
    if vmag > 1e-9:
        nx, ny = vx/vmag, vy/vmag
        cx_c = px + la*nx; cy_c = py + la*ny
        cd = math.sqrt((cx_c-cx)**2 + (cy_c-cy)**2) - r
        # Find breakeven lookahead where carrot hits obstacle surface
        # Solve: |pos + la*dir - obs_center| = r
        # Quadratic in la
        ax, ay = nx, ny
        bx, by = px - cx, py - cy
        A = ax*ax + ay*ay   # = 1 since normalized
        B = 2*(ax*bx + ay*by)
        C = bx*bx + by*by - r*r
        disc = B*B - 4*A*C
        if disc >= 0:
            la1 = (-B - math.sqrt(disc)) / (2*A)
            la2 = (-B + math.sqrt(disc)) / (2*A)
            breakeven = min(x for x in [la1,la2] if x > 0) if any(x>0 for x in [la1,la2]) else None
        else:
            breakeven = None
        be_str = f"la>{breakeven:.3f}m" if breakeven else "never enters"
        inside_str = "YES" if cd < 0 else "no"
        print(f"  {label:<22}  nearest={oname}  d_surf={d_surf:.4f}m  carrot_inside={inside_str}  "
              f"enters_obs_at {be_str}")
    else:
        print(f"  {label:<22}  drift/hover case")

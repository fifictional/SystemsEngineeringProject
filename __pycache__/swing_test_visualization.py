import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.optimize import curve_fit

# ── 1. Load data ──────────────────────────────────────────────
df = pd.read_csv("swing_data.csv")   # columns: time_ms, theta_deg
t  = df["time_ms"].values / 1000.0   # convert to seconds
th = df["theta_deg"].values

# ── 2. Find peaks (local maxima) ──────────────────────────────
peaks, _ = find_peaks(th, height=0.5)   # ignore tiny noise peaks
peak_times  = t[peaks]
peak_angles = th[peaks]

print(f"Found {len(peaks)} peaks")

# ── 3. Logarithmic decrement method ──────────────────────────
# δ = (1/n) * ln(A0 / An)
n = len(peak_angles) - 1
if n > 0:
    delta = (1.0 / n) * np.log(peak_angles[0] / peak_angles[-1])
    zeta  = delta / np.sqrt(4 * np.pi**2 + delta**2)
    print(f"Log decrement δ  = {delta:.6f}")
    print(f"Damping ratio  ζ = {zeta:.6f}")
    if zeta < 0.01:
        print("PASS: ζ < 0.01 (friction is low enough)")
    else:
        print("FAIL: ζ >= 0.01 (too much friction)")

# ── 4. Natural frequency from peak spacing ────────────────────
if len(peak_times) >= 2:
    T_d   = np.mean(np.diff(peak_times))   # damped period (s)
    f_d   = 1.0 / T_d
    omega_d = 2 * np.pi * f_d
    omega_n = omega_d / np.sqrt(1 - zeta**2)
    print(f"\nDamped period    T_d = {T_d:.4f} s")
    print(f"Damped freq      ω_d = {omega_d:.4f} rad/s")
    print(f"Natural freq     ω_n = {omega_n:.4f} rad/s")
    print(f"Theoretical      ω_n = {np.sqrt(9.81/0.3):.4f} rad/s  (l=0.3m)")

# ── 5. Curve fit to decaying sinusoid ─────────────────────────
def decaying_sine(t, A, zeta, omega_n, phi, offset):
    omega_d = omega_n * np.sqrt(1 - zeta**2)
    return A * np.exp(-zeta * omega_n * t) * np.cos(omega_d * t + phi) + offset

try:
    p0 = [peak_angles[0], zeta, omega_n, 0, 0]
    popt, _ = curve_fit(decaying_sine, t, th, p0=p0, maxfev=10000)
    A_fit, z_fit, wn_fit, phi_fit, offset_fit = popt
    print(f"\nCurve fit results:")
    print(f"  ζ_fit  = {z_fit:.6f}")
    print(f"  ω_n fit = {wn_fit:.4f} rad/s")
except Exception as e:
    print(f"Curve fit failed: {e}")

# ── 6. Plot ───────────────────────────────────────────────────
plt.figure(figsize=(12, 5))
plt.plot(t, th, label="Measured θ (deg)", linewidth=1)
plt.plot(peak_times, peak_angles, "ro", label="Peaks", markersize=6)
try:
    plt.plot(t, decaying_sine(t, *popt), "--",
             label=f"Fit: ζ={z_fit:.4f}, ωn={wn_fit:.3f} rad/s", linewidth=2)
except:
    pass
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Pendulum Swing Test — System Identification")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("swing_test_result.png", dpi=150)
plt.show()

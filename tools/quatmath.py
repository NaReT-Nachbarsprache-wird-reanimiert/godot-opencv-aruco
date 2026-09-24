"""Quaternion vocabulary shared by the tools that read marker orientations.

Numpy only, no matplotlib: the live receiver (tcp_receiver.py) uses the same functions as the
offline plots (plot_marker_pos.py), and a diagnostics loop that has to import the plotting stack to
turn a quaternion into three angles would pay that import for nothing.

These lived in plot_marker_pos.py, which was their only consumer until the receiver grew a live
6-DoF readout. Moved rather than copied for the reason plotlib.py exists: the euler ORDER and the
hemisphere rule are conventions, and two copies of a convention are two numbers that can disagree
about the same rotation while both look right. The live overlay and the plot of the recording it
produced now quote one definition.

Everything here takes and returns (N, 4) / (N, 3) arrays in Godot's component order (x, y, z, w),
so a CSV column stack or a single pose wrapped in np.atleast_2d both go straight in.
"""

import numpy as np


def unit(q):
    """(N, 4) quaternions, renormalised.

    They arrive as float32 off the wire and every consumer here assumes unit length -- rotvec_deg
    reads w as cos(angle/2), quat_to_matrix has no normalising step of its own. The epsilon guard is
    stated once, here, rather than at each call site.
    """
    q = np.asarray(q, dtype=float)
    return q / np.maximum(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12)


def quat_hemisphere(q):
    """Flip every sample onto one hemisphere.

    q and -q are the SAME rotation. A log that happens to straddle the sign boundary would
    otherwise get a component-wise median sitting halfway between two IDENTICAL orientations,
    which is not an orientation at all -- and every deviation would then be measured from it.
    """
    sign = np.sign(q @ q[0])
    sign[sign == 0.0] = 1.0
    return q * sign[:, None]


def quat_median(q):
    """Robust central orientation: component-wise median on one hemisphere, renormalised.

    Median rather than mean for the same reason as everywhere else in these tools -- the occasional
    frame where solvePnP picks the wrong branch of the planar ambiguity is off by tens of degrees,
    and an averaged reference would carry a piece of that into every other frame's deviation.
    """
    m = np.median(quat_hemisphere(q), axis=0)
    n = float(np.linalg.norm(m))
    return m / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def quat_mul(a, b):
    """Hamilton product in the (x, y, z, w) component order -- Godot's, so the CSV columns go
    straight in as they came out of Basis.get_rotation_quaternion()."""
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz], axis=-1)


def quat_conj(q):
    out = np.array(q, dtype=float, copy=True)
    out[..., :3] *= -1.0
    return out


def rotvec_deg(q):
    """Quaternion -> rotation vector (axis * angle) in degrees, along the shortest arc.

    A rotation VECTOR rather than euler angles, deliberately. Euler needs a convention stated to be
    read at all, wraps at +-180deg, and degenerates at gimbal lock -- three ways for a plot to show
    a jump that never happened. Taken relative to a median orientation these angles are a couple
    of degrees, nowhere near any of those failure modes, and each component reads simply as "how far
    about that world axis".
    """
    q = np.atleast_2d(np.asarray(q, dtype=float))
    # q and -q are the same rotation; picking w >= 0 picks the <=180deg way round.
    q = q * np.where(q[:, 3:4] < 0.0, -1.0, 1.0)
    w = np.clip(q[:, 3], -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    sin_half = np.sqrt(np.maximum(1.0 - w * w, 0.0))
    axis = np.zeros((len(q), 3))
    ok = sin_half > 1e-9          # at sin_half == 0 the rotation IS identity; axis stays zero
    axis[ok] = q[ok, :3] / sin_half[ok, None]
    return np.degrees(axis * angle[:, None])


def spread_deg(q):
    """RMS angle of a set of orientations about their own median, in degrees.

    THE scalar for "how much is this orientation estimate wobbling" -- one number per window, which
    is what a live readout can show and what a distance/variance experiment correlates against.
    Measured about the MEDIAN orientation rather than the mean for the usual reason: one frame on
    the wrong branch of the planar ambiguity is tens of degrees out, and a mean reference would
    smear that across every other sample's deviation instead of leaving it as the one outlier.

    RMS rather than a per-axis sd because the quantity of interest is rotational scatter as such,
    and splitting it across three world axes makes it depend on how the marker happened to be
    oriented. Read it against solvePnP's own floor on a single small planar marker -- ~2.2deg
    median / 5.2deg p90, see the note at lens_rotation_raw in src/OpenCVProcessor.h.
    """
    q = quat_hemisphere(unit(np.atleast_2d(q)))
    rel = quat_mul(quat_conj(quat_median(q))[None, :], q)
    rv = rotvec_deg(rel)
    return float(np.sqrt(np.mean(np.sum(rv * rv, axis=1))))


def quat_to_matrix(q):
    """(N, 4) quaternions in (x, y, z, w) -> (N, 3, 3) rotation matrices."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=-1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=-1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=-1),
    ], axis=1)


def quat_to_euler_yxz_deg(q, unwrap=True):
    """Quaternion -> euler angles in degrees, in Godot's DEFAULT order (EULER_ORDER_YXZ).

    The order is the whole reason this function exists rather than a one-liner: euler angles are
    meaningless without one, and the useful choice is the one the engine that produced the pose
    uses, so these numbers are directly comparable with Node3D.rotation_degrees in the (remote)
    inspector. A different order applied to the same rotation gives three different numbers, all
    correct, none comparable with anything.

    The gimbal-lock branches are transcribed from Basis::get_euler rather than left to fall over:
    at x = +-90deg the y and z axes coincide, y absorbs the whole remaining rotation and z is
    pinned to zero. Vectorised through np.where, so the degenerate case costs nothing and cannot
    produce a NaN that would silently break the line.

    `unwrap` is for a TRACK of samples: a marker sitting near the +-180deg seam otherwise draws a
    full-scale vertical line every time the angle crosses it -- an artefact of the representation
    that looks exactly like the pose flipping, which is the one thing a plot must not invent.
    Unwrapping moves the values off the canonical range on purpose; read them as a continuous
    track, not as canonical euler angles. Pass False for a single live pose, where there is no
    track to be continuous with and the canonical range is what a readout should show.
    """
    m = quat_to_matrix(q)
    m12 = np.clip(m[:, 1, 2], -1.0, 1.0)
    locked = np.abs(m12) > 1.0 - 1e-7

    x = np.where(locked, np.copysign(np.pi / 2.0, -m12), np.arcsin(-m12))
    y = np.where(locked,
                 np.copysign(1.0, -m12) * np.arctan2(m[:, 0, 1], m[:, 0, 0]),
                 np.arctan2(m[:, 0, 2], m[:, 2, 2]))
    z = np.where(locked, 0.0, np.arctan2(m[:, 1, 0], m[:, 1, 1]))

    eul = np.stack([x, y, z], axis=1)
    return np.degrees(np.unwrap(eul, axis=0) if unwrap else eul)

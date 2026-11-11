"""Find sparse residual returns inside a base_link-aligned chassis volume."""
import numpy as np
from scipy.spatial import cKDTree


def sparse_deck_indices(xyz, keep, base_from_cloud, bounds, radius, min_points):
    low, high = bounds[:3], bounds[3:]
    rotation = base_from_cloud[:3, :3]
    translation = base_from_cloud[:3, 3]
    # Preselect in the lidar frame. Include a halo so a real cluster crossing
    # the box boundary contributes neighbors on both sides.
    center = ((low + high) * .5 - translation) @ rotation
    # An outside center may itself need neighbors one more radius away.
    half = ((high - low) * .5 + 2 * radius) @ np.abs(rotation)
    idx = np.flatnonzero(keep & np.all((xyz >= center-half) & (xyz <= center+half), axis=1))
    local = xyz[idx] @ rotation.T + translation
    inside = np.all((local >= low) & (local <= high), axis=1)
    if not inside.any():
        return idx[inside]
    unique, inverse = np.unique(local, axis=0, return_inverse=True)
    if len(unique) < min_points:
        return idx[inside]
    # Count distinct locations. Keep every member of a dense neighborhood,
    # including its end points, which may have fewer neighbors themselves.
    tree = cKDTree(unique)
    distance, _ = tree.query(unique, k=[min_points], distance_upper_bound=radius)
    centers = unique[np.isfinite(distance[:, 0])]
    if not len(centers):
        return idx[inside]
    nearest, _ = cKDTree(centers).query(unique, distance_upper_bound=radius)
    return idx[inside & ~np.isfinite(nearest[inverse.reshape(-1)])]

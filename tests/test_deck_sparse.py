import numpy as np

from r1pro_waypoint_nav.deck_sparse import sparse_deck_indices


BOUNDS = np.array([-.393, -.337, .41, .234, .337, .70])


def rejected(points, transform=None, keep=None):
    xyz = np.array(points, dtype=float).reshape(-1, 3)
    if transform is None:
        transform = np.eye(4)
    if keep is None:
        keep = np.ones(len(xyz), dtype=bool)
    return sparse_deck_indices(xyz, keep, transform, BOUNDS, .04, 4).tolist()


def test_disjoint_returns_and_repeated_coordinates():
    wire = [[.0501, -.1239, .6126], [.0571, -.1568, .5407], [.0461, -.1668, .6536]]
    assert rejected(wire + [wire[0]]) == [0, 1, 2, 3]
    assert rejected([]) == []


def test_dense_cluster_and_outside_singleton():
    cluster = [[x, 0, .50] for x in [-.031, -.015, 0, .03]]
    assert rejected(cluster + [[.25, 0, .50], [0, .12, .50]]) == [5]
    assert rejected(cluster, keep=np.array([True, False, True, True])) == [0, 2, 3]


def test_neighborhood_straddles_edge_in_sensor_frame():
    transform = np.array([[0., -1., 0., .4], [1., 0., 0., -.2],
                          [0., 0., 1., .1], [0., 0., 0., 1.]])
    base = np.array([[x, 0, .55] for x in [.233, .262, .284, .289]] + [[.2, .18, .55]])
    cloud = (base - transform[:3, 3]) @ transform[:3, :3]
    assert rejected(cloud, transform) == [4]

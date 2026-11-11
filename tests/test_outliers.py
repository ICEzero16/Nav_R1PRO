import unittest
import numpy as np
from r1pro_waypoint_nav.outliers import statistical_inliers,post_filter_mask,range_statistical_inliers


class OutlierTests(unittest.TestCase):
    def test_isolated_point_removed_dense_cluster_retained(self):
        points=np.concatenate([np.random.default_rng(1).normal(0,.01,(100,3)),[[9.,9.,9.]]])
        keep,info=statistical_inliers(points,8,2.)
        self.assertFalse(keep[-1]);self.assertTrue(keep[:-1].all())
    def test_tiny_cloud_preserved(self):
        keep,info=statistical_inliers(np.array([[0.,0,0],[20,20,20]]))
        self.assertTrue(keep.all());self.assertEqual(info['reason'],'insufficient_neighbors_preserved')
    def test_duplicate_points_preserved(self):
        keep,_=statistical_inliers(np.zeros((40,3)));self.assertTrue(keep.all())
    def test_empty(self):
        keep,_=statistical_inliers(np.empty((0,3)));self.assertEqual(len(keep),0)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):statistical_inliers(np.array([[np.nan,0,0]]))
    def test_parameters(self):
        for k in [0,1,129,True,2.5]:
            with self.assertRaises(ValueError):statistical_inliers(np.empty((0,3)),k)
        for s in [0.,-1.,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):statistical_inliers(np.empty((0,3)),16,s)
    def test_only_self_filtered_points_enter_sor(self):
        dense=np.random.default_rng(4).normal(0,.02,(100,3))
        points=np.vstack([[np.nan,0,0],dense,[9.,9.,9.]])
        initial=np.ones(len(points),dtype=bool);initial[0]=False
        keep,removed,info=post_filter_mask(points,initial)
        self.assertFalse(keep[0]);self.assertFalse(removed[0]);self.assertTrue(removed[-1])
        self.assertTrue(np.array_equal(keep|removed,initial));self.assertFalse((keep&removed).any())
    def test_disabled_is_exact_passthrough(self):
        initial=np.array([False,True,True]);keep,removed,_=post_filter_mask(np.array([[np.nan,0,0],[1,2,3],[9,8,7]]),initial,False)
        self.assertTrue(np.array_equal(keep,initial));self.assertFalse(removed.any())
    def test_rigid_transform_invariant(self):
        p=np.random.default_rng(6).normal(size=(300,3));a,_=statistical_inliers(p)
        b,_=statistical_inliers(p[:,[1,0,2]]*[-1,1,1]+[3,5,7])
        self.assertTrue(np.array_equal(a,b))


class RangeOutlierTests(unittest.TestCase):
    def scene(self):
        rng=np.random.default_rng(22)
        near=rng.normal(0,.025,(2000,3))+[1,0,0]
        far=rng.normal(0,.25,(200,3))+[8,0,0]
        return np.vstack([near,far,[0,0,8]])
    def test_far_cluster_preserved_isolated_point_removed(self):
        p=self.scene();old,_=statistical_inliers(p);new,info=range_statistical_inliers(p)
        self.assertGreater(new[2000:-1].sum(),old[2000:-1].sum())
        self.assertFalse(new[-1]);self.assertFalse((old&~new).any())
        thresholds=[b['threshold_m'] for b in info['bins'] if b['threshold_m'] is not None]
        self.assertTrue(np.all(np.diff(thresholds)>=0))
    def test_small_bin_preserved(self):
        p=np.vstack([np.zeros((100,3)),[30,0,0]])
        keep,info=range_statistical_inliers(p)
        self.assertTrue(keep[-1]);self.assertEqual(info['bins'][-1]['reason'],'insufficient_bin_points_preserved')
    def test_rotation_about_sensor_origin_invariant(self):
        p=self.scene();a,_=range_statistical_inliers(p);b,_=range_statistical_inliers(p[:,[1,0,2]]*[-1,1,1])
        self.assertTrue(np.array_equal(a,b))
    def test_bin_boundaries_and_global_neighbor_search(self):
        p=np.vstack([np.tile([1.99,0,0],(80,1)),np.tile([2.,0,0],(80,1)),np.tile([2.01,0,0],(80,1))])
        keep,info=range_statistical_inliers(p)
        self.assertTrue(keep.all());self.assertEqual(info['bins'][0]['input_points'],80)
        self.assertEqual(info['bins'][1]['input_points'],160)
    def test_validation(self):
        for edges in [[],[0,1],[2,1],[2,2],[float('inf')],[float('nan')],[[1,2]]]:
            with self.assertRaises(ValueError):range_statistical_inliers(np.empty((0,3)),range_edges=edges)
        for n in [True,2,64.5]:
            with self.assertRaises(ValueError):range_statistical_inliers(np.empty((0,3)),min_bin_points=n)
        with self.assertRaises(ValueError):range_statistical_inliers([[np.nan,0,0]])
    def test_original_record_masks(self):
        p=np.vstack([[np.nan,0,0],self.scene()]);initial=np.ones(len(p),dtype=bool);initial[0]=False
        final,rejected,_=post_filter_mask(p,initial,True,16,3.,method='range')
        self.assertTrue(np.array_equal(final|rejected,initial));self.assertFalse((final&rejected).any())
    def test_empty_and_small_input(self):
        for p in [np.empty((0,3)),np.zeros((10,3))]:
            keep,_=range_statistical_inliers(p);self.assertTrue(keep.all())


if __name__=='__main__':unittest.main()

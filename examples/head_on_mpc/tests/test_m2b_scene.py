import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2b_scene import build_scene,build_routes,check_scenario,audit,CARS

class SceneTests(unittest.TestCase):
 def test_fixed_scene_and_all_pairs(self):
  s=build_scene();r=build_routes(s);q=check_scenario(s,r)
  self.assertTrue(q['passed']);self.assertEqual(q['entry_distances_m'],dict(A1=25,B1=27,A2=29,B2=30));self.assertAlmostEqual(q['initial_minimum_gap_m'],4.2)
  self.assertEqual(len(q['initial']['pair_gaps_m']),6)
  self.assertEqual(s['road_rectangles'],[[-52,52,-1.4,1.4],[-52,-10,-5.5,5.5],[10,52,-5.5,5.5]])
 def test_DONE_obstacles_not_removed(self):
  s=build_scene();z={c:np.array([s['goals'][c]]) for c in CARS};z['A2']=z['A1'].copy()
  self.assertFalse(audit(z,s)['passed']);self.assertEqual(len(audit(z,s)['pair_gaps_m']),6)

if __name__=='__main__':unittest.main()

import copy,json,sys,tempfile,shutil,unittest
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
from validate_scene import validate
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)/'S01_straight';shutil.copytree(ROOT/'scenes/S01_straight',self.root)
 def tearDown(self):self.tmp.cleanup()
 def edit(self,name,fn):
  f=self.root/name;data=yaml.safe_load(f.read_text());fn(data);f.write_text(yaml.safe_dump(data))
 def bad(self):
  with self.assertRaises(Exception):validate(self.root)
 def test_good(self):self.assertTrue(validate(self.root)['passed'])
 def test_duplicate_anchor(self):self.edit('scenario.yaml',lambda d:d['anchors'].append(d['anchors'][0]));self.bad()
 def test_duplicate_geometry(self):self.edit('scene_semantics.yaml',lambda d:d['geometry'].append(d['geometry'][0]));self.bad()
 def test_version_mismatch(self):self.edit('demo_route.yaml',lambda d:d.update(scene_version='9.0.0'));self.bad()
 def test_path_escape(self):self.edit('scenario.yaml',lambda d:d.update(world='../outside.sdf'));self.bad()
 def test_online_path(self):self.edit('scenario.yaml',lambda d:d.update(world='https://example.com/model.sdf'));self.bad()
 def test_missing_asset(self):self.edit('scenario.yaml',lambda d:d['assets'].append('assets/missing.dae'));self.bad()
 def test_wrong_frame(self):self.edit('scenario.yaml',lambda d:d.update(global_frame='odom'));self.bad()
 def test_nonidentity_map(self):self.edit('scenario.yaml',lambda d:d['map_to_world'].update(xyz_m=[1,0,0]));self.bad()
 def test_self_cross_polygon(self):self.edit('scene_semantics.yaml',lambda d:d['zones'][0].update(polygon_xy_m=[[0,0],[2,2],[0,2],[2,0]]));self.bad()
 def test_duplicate_time(self):self.edit('demo_route.yaml',lambda d:d['waypoints'][1].update(time_s=0));self.bad()
 def test_wall_crossing(self):self.edit('demo_route.yaml',lambda d:d['waypoints'][1].update(xyz_m=[4,18,.5]));self.bad()
 def test_semantic_box_mismatch(self):self.edit('scene_semantics.yaml',lambda d:d['geometry'][0].update(size_m=[.3,20,3]));self.bad()
 def test_nan_config(self):self.edit('scenario.yaml',lambda d:d['anchors'][0].update(position_m=[float('nan'),0,0]));self.bad()
 def test_unknown_field(self):self.edit('scenario.yaml',lambda d:d.update(dust_active=True));self.bad()
 def test_duplicate_yaml_key(self):
  f=self.root/'scenario.yaml';f.write_text(f.read_text()+'\nscene_id: S02_corner\n');self.bad()
if __name__=='__main__':
 suite=unittest.defaultTestLoader.loadTestsFromTestCase(Tests);r=unittest.TextTestRunner(verbosity=2).run(suite)
 out=ROOT/'tests/results';out.mkdir(exist_ok=True);(out/'validator_tests.json').write_text(json.dumps(dict(passed=r.wasSuccessful(),count=r.testsRun,failures=len(r.failures),errors=len(r.errors)),indent=2));sys.exit(not r.wasSuccessful())

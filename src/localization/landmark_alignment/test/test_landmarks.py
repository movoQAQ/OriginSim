import importlib.util,unittest,numpy as np,rclpy
from pathlib import Path
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
spec=importlib.util.spec_from_file_location('align',str(Path(__file__).resolve().parents[1] / 'landmark_alignment.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Planes(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def test_three_walls_are_supported_but_parallel_or_diagonal_clouds_are_not(self):
  u=np.linspace(.2,15.8,200);v=np.linspace(.2,28.8,200)
  left=np.c_[np.zeros(200),u];right=np.c_[np.full(200,29),u]
  top=np.c_[v,np.full(200,16)]
  planes=m.learn_planes(np.r_[left,right,top])
  self.assertIsNotNone(planes);self.assertEqual(len(planes),3)
  self.assertIsNone(m.learn_planes(np.r_[left,right]))
  self.assertIsNone(m.learn_planes(np.c_[v,.5*v]))
 def test_fixed_landmarks_recover_drift_and_require_stationary_bootstrap(self):
  n=m.Alignment();t=n.initial;u=np.linspace(.2,15.8,200)
  world=np.r_[np.c_[np.zeros(200),u],np.c_[np.full(200,29),u],np.c_[np.linspace(.2,28.8,200),np.zeros(200)],np.c_[np.linspace(.2,28.8,200),np.full(200,16)]]
  def cloud(t):
   c,s=np.cos(t[2]),np.sin(t[2]);p=(world-t[:2])@np.array([[c,-s],[s,c]])
   return create_cloud_xyz32(Header(frame_id='odom'),np.c_[p,np.full(len(p),6)])
  for _ in range(10):n.cloud(cloud(t))
  self.assertIsNone(n.planes)
  n.stationary=True
  for _ in range(8):n.cloud(cloud(t))
  self.assertIsNotNone(n.planes)
  target=t+np.array([.12,-.08,.004])
  for _ in range(15):n.cloud(cloud(target))
  np.testing.assert_allclose(n.transform,target,atol=.005)
  before=n.transform.copy()
  empty=create_cloud_xyz32(Header(frame_id='odom'),np.zeros((10,3)))
  n.cloud(empty);np.testing.assert_array_equal(n.transform,before);n.destroy_node()
if __name__=='__main__':unittest.main()

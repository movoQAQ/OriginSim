#!/usr/bin/env python3
"""Keep map->odom anchored to stationary arena perimeter planes, using LiDAR only.

The initial transform is operator calibration. Tall planes are learned while
stationary, then retained as fixed landmarks; no simulator pose is consumed.
"""
import numpy as np
from scipy.optimize import least_squares
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py.point_cloud2 import read_points_numpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def learn_planes(points):
 # A nearby wall can be above the LiDAR vertical FOV. Require three supported
 # planes spanning both normal directions, rather than four visible extrema.
 planes=[]
 for axis in (0,1):
  values=points[:,axis].copy()
  for _ in range(2):
   if len(values)<80:break
   bins=np.arange(values.min()-.01,values.max()+.04,.02)
   counts,edges=np.histogram(values,bins)
   peak=int(np.argmax(counts));bound=.5*(edges[peak]+edges[peak+1])
   mask=np.abs(points[:,axis]-bound)<.04
   normal=points[mask,axis];tangent=points[mask,1-axis]
   if (len(normal)>=max(80,len(points)*.005) and np.std(normal)<.025 and
       np.quantile(tangent,.95)-np.quantile(tangent,.05)>2.):
    planes.append((axis,float(np.median(normal))))
   values=values[np.abs(values-bound)>1.]
 return planes if len(planes)>=3 and len({axis for axis,_ in planes})==2 else None

class Alignment(Node):
 def __init__(self):
  super().__init__('landmark_alignment')
  self.initial=np.array([self.declare_parameter(k,v).value for k,v in [('X',3.4),('Y',9.5),('Rotation',-1.095)]])
  self.z=self.declare_parameter('Z',.8758).value
  self.min_z=self.declare_parameter('landmark_min_height',4.).value
  self.transform=self.initial.copy();self.planes=None;self.boot=[];self.stationary=False
  self.odom_sub=self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)
  self.br=TransformBroadcaster(self)
  self.sub=self.create_subscription(PointCloud2,'/cloud_registered',self.cloud,qos_profile_sensor_data)
  self.timer=self.create_timer(.02,self.broadcast)
 def odometry(self,m):
  self.stationary=np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)<.05 and abs(m.twist.twist.angular.z)<.05
 def world(self,xy,t):
  c,s=np.cos(t[2]),np.sin(t[2]);return xy@np.array([[c,s],[-s,c]])+t[:2]
 def broadcast(self):
  if self.planes is None:return
  t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.header.stamp=self.get_clock().now().to_msg()
  t.transform.translation.x=float(self.transform[0]);t.transform.translation.y=float(self.transform[1]);t.transform.translation.z=float(self.z)
  t.transform.rotation.z=float(np.sin(self.transform[2]/2));t.transform.rotation.w=float(np.cos(self.transform[2]/2));self.br.sendTransform(t)
 def cloud(self,m):
  a=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True)
  a=a[a[:,2]+self.z>self.min_z,:2].astype(float)
  if len(a)<100:return
  if self.planes is None:
   if not self.stationary:self.boot=[];return
   self.boot.append(self.world(a,self.initial))
   if len(self.boot)<8:return
   planes=learn_planes(np.concatenate(self.boot))
   if planes is None:self.boot=[];return
   self.planes=planes;self.boot=[];self.get_logger().info('Fixed LiDAR perimeter landmarks initialized')
   return
  # Associate only to the fixed learned planes. Reject unsupported or large updates.
  w=self.world(a,self.transform);res=np.stack([np.abs(w[:,axis]-d) for axis,d in self.planes],axis=1)
  labels=np.argmin(res,axis=1);keep=np.min(res,axis=1)<.35
  a=a[keep][::2];labels=labels[keep][::2]
  if len(a)<100 or len(set(labels))<3:return
  def residual(t):
   w=self.world(a,t)
   return np.array([w[i,self.planes[label][0]]-self.planes[label][1] for i,label in enumerate(labels)])
  old=self.transform.copy()
  result=least_squares(residual,old,bounds=(old-[.2,.2,.02],old+[.2,.2,.02]),loss='soft_l1',f_scale=.02,max_nfev=12)
  if not result.success or np.median(np.abs(residual(result.x)))>.06:return
  if np.linalg.norm(result.x[:2]-self.initial[:2])>1. or abs(result.x[2]-self.initial[2])>.15:return
  # Limit correction per cloud to keep controller-frame changes small.
  self.transform+=np.clip(result.x-old,[-.025,-.025,-.003],[.025,.025,.003])

if __name__=='__main__':
 rclpy.init();n=Alignment()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()

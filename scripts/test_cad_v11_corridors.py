#!/usr/bin/env python3
"""Bounded live Nav2 route test; preserves Gazebo/world and cancels on failure."""
import argparse,json,math,time,threading,xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from tf2_ros import Buffer,TransformListener
from std_srvs.srv import Empty

parser=argparse.ArgumentParser();parser.add_argument('--tag',required=True);parser.add_argument('--route',default='loop');parser.add_argument('--world-file',default='corridor_speed_trial.world');parser.add_argument('--timeout',type=float,default=100);parser.add_argument('--allow-guarded-pause',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parents[1];out=root/'output/cad_v11_corridor_tuning';out.mkdir(parents=True,exist_ok=True)
world=ET.parse(root/'src/physical_ai_robot_description/worlds'/args.world_file).getroot()
obstacles=[]
for m in world.findall('.//world/model'):
 size=m.findtext('.//collision/geometry/box/size')
 if not size:continue
 x,y,z,*_=map(float,m.findtext('pose','0 0 0 0 0 0').split());sx,sy,sz=map(float,size.split())
 obstacles.append([(x-sx/2,y-sy/2),(x+sx/2,y-sy/2),(x+sx/2,y+sy/2),(x-sx/2,y+sy/2)])
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def point_segment(p,a,b):
 dx,dy=b[0]-a[0],b[1]-a[1];t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
 return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
def clearance(poly,other):
 separated=False
 for shape in (poly,other):
  for i,a in enumerate(shape):
   b=shape[(i+1)%4];nx,ny=-(b[1]-a[1]),b[0]-a[0]
   aa=[nx*x+ny*y for x,y in poly];bb=[nx*x+ny*y for x,y in other]
   if max(aa)<min(bb) or max(bb)<min(aa):separated=True
 if not separated:return 0.
 return min(point_segment(p,s[i],s[(i+1)%4]) for pts,s in ((poly,other),(other,poly)) for p in pts for i in range(4))
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('cad_corridor_test',parameter_overrides=[Parameter('use_sim_time',value=True)])
tf=Buffer();listener=TransformListener(tf,node);client=ActionClient(node,NavigateToPose,'navigate_to_pose')
pause_client=node.create_client(Empty,'/pause_physics')
state={'odom':None,'seen':0.,'max_receipt_gap':0.,'safe_seen':0.,'safe_v':0.,'safe_w':0.};logs=[]
def on_odom(m):
 now=time.monotonic()
 if state['seen']:state['max_receipt_gap']=max(state['max_receipt_gap'],now-state['seen'])
 state.update(odom=m,seen=now)
node.create_subscription(Odometry,'/odom',on_odom,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
def on_safe(m):state.update(safe_seen=time.monotonic(),safe_v=m.linear.x,safe_w=m.angular.z)
node.create_subscription(Twist,'/cmd_vel_safe',on_safe,10)
node.create_subscription(Log,'/rosout',lambda m:logs.append((time.monotonic(),m.name,m.msg)) if m.level>=30 or (m.name=='behavior_server' and 'Running' in m.msg) else None,100)
# Keep telemetry reception independent of geometry/report calculations.
executor=SingleThreadedExecutor();executor.add_node(node)
receiver=threading.Thread(target=executor.spin,daemon=True);receiver.start()
report={'tag':args.tag,'world':args.world_file,'payload_kg':0,'model':'MDH100 v11','envelope_m':[[-.225,-.245],[.365,-.245],[.365,.245],[-.225,.245]],'allows_verified_guarded_pause':args.allow_guarded_pause,'steps':[],'passed':False};active=None
def save(): (out/(args.tag+'.json')).write_text(json.dumps(report,indent=2),'utf-8')
def wait(predicate,seconds):
 end=time.monotonic()+seconds
 while not predicate():
  assert time.monotonic()<end,'wait timed out'
  time.sleep(.02)
routes={'loop':[('setup_north',.4,3.1,-math.pi/2),('north_to_east',3.1,.35,0),('east_to_south',.35,-3.1,-math.pi/2),('south_to_west',-3.1,-.35,math.pi),('west_to_north',.4,3.1,math.pi/2)],
 'home':[('return_center',0.,0.,0.)],
 'narrow_roundtrip':[('setup_center',0.,0.,0.),('east_out',3.1,.35,0.),('east_back',0.,0.,math.pi)],
 'corner':[('setup_north',.4,3.1,-math.pi/2),('north_to_east',3.1,.35,0)],
 'east_obstacle':[('east_out',3.1,.35,0),('east_back',.0,.0,3.141592653589793)],
 'east_return':[('setup_east',3.1,.35,math.pi),('east_to_north',.4,3.1,math.pi/2)]}
try:
 wait(lambda:state['odom'] is not None and tf.can_transform('map','odom',rclpy.time.Time()),15);assert client.wait_for_server(timeout_sec=10)
 for name,x,y,heading in routes[args.route]:
  t=tf.lookup_transform('map','odom',rclpy.time.Time()).transform;a=yaw(t.rotation)
  goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
  goal.pose.pose.position.x=t.translation.x+math.cos(a)*x-math.sin(a)*y;goal.pose.pose.position.y=t.translation.y+math.sin(a)*x+math.cos(a)*y
  goal.pose.pose.orientation.z=math.sin((a+heading)/2);goal.pose.pose.orientation.w=math.cos((a+heading)/2)
  f=client.send_goal_async(goal);wait(f.done,10);active=f.result();assert active.accepted,'goal rejected';f=active.get_result_async()
  start=time.monotonic();sim_start=state['odom'].header.stamp.sec+state['odom'].header.stamp.nanosec*1e-9;last_print=start;prev=None;rows=[];distance=0.;min_gap=99.;max_tilt=0.;reason=None;state['max_receipt_gap']=0.;guarded_pauses=[];pause=None
  print('START',name,flush=True)
  while not f.done():
   time.sleep(.02);now=time.monotonic();m=state['odom'];p=m.pose.pose.position;q=m.pose.pose.orientation;a=yaw(q)
   tilt=math.degrees(math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y)))))
   poly=[(p.x+math.cos(a)*xx-math.sin(a)*yy,p.y+math.sin(a)*xx+math.cos(a)*yy) for xx,yy in [(-.225,-.245),(.365,-.245),(.365,.245),(-.225,.245)]]
   gap=min(clearance(poly,o) for o in obstacles);min_gap=min(min_gap,gap);max_tilt=max(max_tilt,tilt)
   if prev:distance+=math.hypot(p.x-prev[0],p.y-prev[1])
   prev=(p.x,p.y)
   if not rows or now-start-rows[-1][0]>.2:rows.append([round(now-start,3),p.x,p.y,a,m.twist.twist.linear.x,m.twist.twist.angular.z,gap])
   if now-last_print>10:print('PROGRESS',name,round(now-start,1),[round(p.x,2),round(p.y,2)],'clearance',round(gap,3),flush=True);last_print=now
   age=now-state['seen'];state['max_receipt_gap']=max(state['max_receipt_gap'],age)
   guarded=now-state['safe_seen']<.2 and abs(state['safe_v'])<1e-6 and abs(state['safe_w'])<1e-6
   if args.allow_guarded_pause and age>.5 and guarded:
    if pause is None:pause={'start_s':round(now-start,3),'max_odom_age_s':age};guarded_pauses.append(pause)
    pause['max_odom_age_s']=max(pause['max_odom_age_s'],age)
   elif age<.3:pause=None
   if tilt>10:reason='tilt limit'
   elif args.allow_guarded_pause and age>.5 and not guarded:reason='stale odometry without fresh verified zero command'
   elif age>(2. if args.allow_guarded_pause else 1.):reason='stale odometry'
   elif gap<.04:reason='conservative envelope clearance below 40mm'
   elif now-start>args.timeout:reason='timeout'
   if reason:break
  if reason:
   if 'clearance' in reason or 'tilt' in reason:
    # Stop the test physics before cancellation/deceleration can consume the
    # remaining margin. This is a harness stop, not a robot safety claim.
    assert pause_client.wait_for_service(timeout_sec=2)
    paused=pause_client.call_async(Empty.Request());wait(paused.done,3)
    report['physics_paused_on_abort']=True
   cancel=active.cancel_goal_async();wait(cancel.done,5)
  status=f.result().status if f.done() else None;active=None
  row={'name':name,'goal_world':[x,y,heading],'status':status,'reason':reason,'duration_s':round(time.monotonic()-start,2),'simulation_duration_s':round(state['odom'].header.stamp.sec+state['odom'].header.stamp.nanosec*1e-9-sim_start,2),'path_m':round(distance,3),'min_envelope_clearance_m':round(min_gap,4),'max_tilt_deg':max_tilt,'samples_t_x_y_yaw_v_w_gap':rows,'warnings':list(dict.fromkeys(n+': '+msg for tt,n,msg in logs if tt>=start))}
  row['max_odom_receipt_gap_s']=state['max_receipt_gap']
  row['verified_guarded_pauses']=guarded_pauses
  row['control_rate_miss_count']=sum('Control loop missed' in msg for tt,n,msg in logs if tt>=start)
  report['steps'].append(row);save();print('END',json.dumps({k:v for k,v in row.items() if k not in ('samples_t_x_y_yaw_v_w_gap','warnings')}),flush=True)
  assert not reason and status==4,'stage failed'
  until=time.monotonic()+.7
  while time.monotonic()<until:time.sleep(.02)
 report['passed']=True
except (Exception,KeyboardInterrupt) as e:report['error']=str(e) or 'operator interrupted trial';print('STOP',report['error'],flush=True)
finally:
 if active:
  f=active.cancel_goal_async()
  try:wait(f.done,5)
  except AssertionError:pass
 save();executor.shutdown();receiver.join(timeout=2);node.destroy_node();rclpy.shutdown()
print('REPORT',str(out/(args.tag+'.json')),flush=True)

if not report['passed']:raise SystemExit(1)

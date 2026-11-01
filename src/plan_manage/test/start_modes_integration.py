#!/usr/bin/env python3
"""Exercise the real FSM binary with synthetic MAVROS on a private ROS master."""
import os, subprocess, time, threading, math, yaml, signal, json
from pathlib import Path
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11447'
os.environ['ROS_IP']='127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped,PointStamped
from mavros_msgs.msg import State,EstimatorStatus,AttitudeTarget
from mavros_msgs.srv import SetMode,SetModeResponse,CommandBool,CommandBoolResponse
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import FsmState
from std_msgs.msg import Float64
import rospkg
ws=Path(rospkg.RosPack().get_path('plan_manage')).parents[1]; results=[]; processes=[]
core_log=open('/tmp/start_modes_core.log','w')
core=subprocess.Popen(['roscore','-p','11447'],stdout=core_log,stderr=subprocess.STDOUT,start_new_session=True)
processes.append(core)
time.sleep(2)
rospy.init_node('start_modes_integration',anonymous=True)
feed={'mode':'POSCTL','armed':True,'valid':False,'xyz':[8,-4,-8],'yaw':math.pi/2,'velocity':[0,0,0]}
seen={'origin':[],'position':[],'state':[],'odom':[],'yaw':[],'mode_request':[],'arm_request':[]}
subscribers=[]
for key,topic,typ in [('origin','/local_frame/origin',PointStamped),('position','/mavros/setpoint_position/local',PoseStamped),('state','/fsm/fsm_state',FsmState),('odom','/nmpc/uavStateEstimate',Odometry),('yaw','/local_frame/initial_yaw',Float64)]:
 subscribers.append(rospy.Subscriber(topic,typ,lambda msg,k=key:seen[k].append(msg)))
pubs=[rospy.Publisher('/mavros/state',State,queue_size=5),rospy.Publisher('/mavros/estimator_status',EstimatorStatus,queue_size=5),rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=5),rospy.Publisher('/mavros/local_position/velocity_local',TwistStamped,queue_size=5)]
def mode_service(req):
 seen['mode_request'].append(req.custom_mode);feed['mode']=req.custom_mode
 return SetModeResponse(mode_sent=True)
def arm_service(req):
 seen['arm_request'].append(req.value);feed['armed']=req.value
 return CommandBoolResponse(success=True,result=0)
services=[rospy.Service('/mavros/set_mode',SetMode,mode_service),rospy.Service('/mavros/cmd/arming',CommandBool,arm_service)]
stop=False
def publish():
 while not stop:
  st=State(connected=True,armed=feed['armed'],mode=feed['mode']); st.header.stamp=rospy.Time.now()
  est=EstimatorStatus();est.header.stamp=st.header.stamp
  for name in ['pos_horiz_rel_status_flag','pos_horiz_abs_status_flag','pos_vert_abs_status_flag']:setattr(est,name,feed['valid'])
  pose=PoseStamped();pose.header.stamp=st.header.stamp;pose.header.frame_id='map'
  pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=feed['xyz']
  pose.pose.orientation.w=math.cos(feed['yaw']/2);pose.pose.orientation.z=math.sin(feed['yaw']/2)
  vel=TwistStamped();vel.header=pose.header
  vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z=feed['velocity']
  for pub,msg in zip(pubs,[st,est,pose,vel]):pub.publish(msg)
  time.sleep(.01)
threading.Thread(target=publish,daemon=True).start()
def check(condition,label):
 assert condition,label
 results.append(label);print('PASS:',label,flush=True)
def wait_for(predicate,timeout=4):
 end=time.time()+timeout
 while time.time()<end:
  if predicate():return True
  time.sleep(.02)
 return False
def start(mode, confirmation=True):
 for key in seen:seen[key].clear()
 params=yaml.safe_load((ws/'src/plan_manage/config/interception_sim_fsm.yaml').read_text())
 params['interception']['start_mode']=mode
 params['interception']['exec']['require_enter_confirmation']=confirmation
 params['interception']['idle']['start_z']=2
 params['interception']['offboard']['warmup_setpoints']=25
 params['interception']['offboard']['mode_request_interval']=.2
 params['manager']={'has_target_pos':False}
 rospy.set_param('/mode_test',params)
 log=open('/tmp/start_modes_'+mode+'.log','w')
 p=subprocess.Popen([str(ws/'devel/lib/plan_manage/fsm_node'),'__name:=mode_test'],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,start_new_session=True)
 processes.append(p);time.sleep(.5);return p
def finish(p):
 p.send_signal(signal.SIGINT)
 try:p.wait(timeout=3)
 except subprocess.TimeoutExpired:p.kill();p.wait()
 time.sleep(.3)
try:
 p=start('airborne_takeover')
 check(not seen['origin'] and not seen['position'],'airborne: invalid first pose does not lock origin or stream')
 feed['valid']=True;time.sleep(.8)
 check(not seen['origin'] and bool(seen['position']),'airborne: valid POSCTL streams measured position without locking origin')
 check(abs(seen['position'][-1].pose.position.z+8)<1e-5,'airborne: negative raw altitude does not block pre-entry stream')
 feed['mode']='OFFBOARD'
 check(wait_for(lambda:any(m.state_name=='WAIT_TARGET' for m in seen['state'])),'airborne: stable OFFBOARD enters WAIT_TARGET')
 check(abs(seen['origin'][-1].point.z+8)<1e-5,'airborne: origin captured at handover')
 check(wait_for(lambda: seen['odom'] and abs(seen['odom'][-1].pose.pose.position.z)<1e-5),'airborne: control z becomes zero')
 check(abs(seen['yaw'][-1].data-math.pi/2)<1e-5,'airborne: heading latched at handover')
 seen['mode_request'].clear();feed['xyz']=[8,-4,-11];time.sleep(.3)
 check(seen['state'][-1].state_name=='WAIT_TARGET' and 'STABILIZED' not in seen['mode_request'],
       'airborne: descent below the old relative altitude limit does not trigger STABILIZED')
 old_origin_count=len(seen['origin']);feed['mode']='POSCTL';time.sleep(.2)
 feed['xyz']=[30,-20,-20];feed['yaw']=-math.pi/2;time.sleep(.8)
 check(seen['state'][-1].state_name=='IDLE','airborne: pilot exit returns IDLE')
 check(len(seen['origin'])==old_origin_count,'airborne: repositioning does not reset origin before next handover')
 check(abs(seen['position'][-1].pose.position.z+20)<1e-5,'airborne: previous origin does not block a lower new entry')
 feed['mode']='OFFBOARD'
 check(wait_for(lambda:len(seen['origin'])>old_origin_count),'airborne: next OFFBOARD locks a new origin')
 check(abs(seen['origin'][-1].point.x-30)<1e-5 and abs(seen['origin'][-1].point.z+20)<1e-5,'airborne: new origin uses new position')
 feed['mode']='POSCTL';time.sleep(.2);feed['velocity']=[2,0,0];time.sleep(.2);feed['mode']='OFFBOARD';time.sleep(.3)
 check(feed['mode']=='POSCTL' and 'POSCTL' in seen['mode_request'],'airborne: moving entry is rejected to POSCTL')
 check(not seen['arm_request'],'airborne: never sends arming command')
 finish(p)
 feed.update(mode='POSCTL',armed=False,valid=False,xyz=[8,-4,-8],yaw=math.pi/2,velocity=[0,0,0])
 p=start('ground_takeoff');p.stdin.write('\n');p.stdin.flush();time.sleep(.15)
 feed['valid']=True;time.sleep(.9)
 check(not seen['origin'] and not seen['mode_request'],'ground: premature Enter is discarded instead of queued')
 p.stdin.write('\n');p.stdin.flush()
 check(wait_for(lambda:bool(seen['origin'])),'ground: valid Enter locks origin')
 check(abs(seen['origin'][-1].point.z+8)<1e-5,'ground: origin is current stable ground pose')
 check(wait_for(lambda:bool(seen['arm_request']),5),'ground: warmup followed by OFFBOARD and arming')
 check(len(seen['position'])>=25,'ground: sufficient setpoints before OFFBOARD request')
 count=len(seen['origin']);p.stdin.write('\n');p.stdin.flush();time.sleep(.1)
 check(len(seen['origin'])==count,'ground: Enter during takeoff does not reset origin')
 feed['xyz']=[8,-4,-6]
 check(wait_for(lambda:seen['state'][-1].state_name=='WAIT_TARGET'),'ground: reaches configured relative takeoff height')
 feed['mode']='POSCTL';feed['armed']=False;time.sleep(.3)
 feed['xyz']=[12,3,-10];time.sleep(.8);p.stdin.write('\n');p.stdin.flush()
 check(wait_for(lambda:len(seen['origin'])>count),'ground: after disarm a new Enter locks a new ground origin')
 check(abs(seen['origin'][-1].point.x-12)<1e-5,'ground: new experiment uses new ground position')
 finish(p)
 feed.update(mode='POSCTL',armed=False,valid=False,xyz=[8,-4,-8],yaw=0,velocity=[0,0,0])
 p=start('ground_takeoff',confirmation=False)
 check(not seen['origin'],'simulation: automatic profile waits for valid localization')
 feed['valid']=True
 check(wait_for(lambda:bool(seen['origin'])),'simulation: automatic ground profile locks origin after stability window')
 check(wait_for(lambda:bool(seen['arm_request']),5),'simulation: existing automatic takeoff profile still starts')
 finish(p)
 # Bad mode must terminate with an explicit error rather than silently choosing a mode.
 rospy.set_param('/bad_mode_test/interception/start_mode','invalid_mode')
 bad=subprocess.run([str(ws/'devel/lib/plan_manage/fsm_node'),'__name:=bad_mode_test'],capture_output=True,text=True,timeout=4)
 check('invalid start_mode' in bad.stdout+bad.stderr,'configuration: invalid mode rejected explicitly')
 # Validate the actual bridge ATTITUDE output, without running a pursuit solver.
 feed.update(mode='POSCTL',armed=True,valid=True,velocity=[0,0,0])
 bridge_params=yaml.safe_load((ws/'src/isb_nmpc/config/holydrone/real_flight/isb_nmpc_real_outdoor_10ms_30degree_from_sim.yaml').read_text())
 bridge_params.update(manager={'has_target_pos':False},enable_log=False,attitude_target_topic='/test/attitude')
 rospy.set_param('/bridge_test',bridge_params)
 camera=yaml.safe_load((ws/'src/camera_driver/config/camera/camera_params_usb_imx577-1_30degree.yaml').read_text())
 for key,value in camera.items():rospy.set_param('/'+key,value)
 attitudes=[]
 subscribers.append(rospy.Subscriber('/test/attitude',AttitudeTarget,lambda m:attitudes.append(m)))
 yaw_pub=rospy.Publisher('/local_frame/initial_yaw',Float64,queue_size=1,latch=True)
 state_pub=rospy.Publisher('/nmpc/uavStateEstimate',Odometry,queue_size=5)
 fsm_pub=rospy.Publisher('/fsm/fsm_state',FsmState,queue_size=5)
 bridge_log=open('/tmp/start_modes_bridge.log','w')
 bridge=subprocess.Popen([str(ws/'devel/lib/isb_nmpc/so3_mpc_bridge_node'),'__name:=bridge_test'],stdout=bridge_log,stderr=subprocess.STDOUT,start_new_session=True)
 processes.append(bridge);time.sleep(.8)
 check(bridge.poll() is None,'bridge: starts successfully with current flight parameters')
 idle=FsmState();idle.state_name='IDLE'
 for _ in range(10):fsm_pub.publish(idle);time.sleep(.03)
 feed['mode']='OFFBOARD'
 for offset,relative in [(math.pi/2,.2),(-math.pi/2,-.3)]:
  yaw_pub.publish(Float64(data=offset));time.sleep(.1);attitudes.clear()
  end=time.time()+.6
  while time.time()<end:
   odom=Odometry();odom.header.stamp=rospy.Time.now();odom.header.frame_id='map'
   odom.pose.pose.orientation.w=math.cos(relative/2);odom.pose.pose.orientation.z=math.sin(relative/2)
   state_pub.publish(odom)
   fsm=FsmState();fsm.state_name='EXEC_INTERCEPTION';fsm_pub.publish(fsm)
   time.sleep(.02)
  check(bool(attitudes),'bridge: fallback ATTITUDE output received')
  q=attitudes[-1].orientation
  actual=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  error=math.atan2(math.sin(actual-offset-relative),math.cos(actual-offset-relative))
  check(abs(error)<1e-5,'bridge: ATTITUDE yaw converted to MAVROS world frame, offset='+str(offset))
 finish(bridge)
 rospy.set_param('/old_switch_test/interception/manual_offboard_takeover',True)
 old=subprocess.run([str(ws/'devel/lib/plan_manage/fsm_node'),'__name:=old_switch_test'],capture_output=True,text=True,timeout=4)
 check('manual_offboard_takeover was removed' in old.stdout+old.stderr,'configuration: removed boolean rejected with migration message')
 Path('/tmp/start_modes_test_results.json').write_text(json.dumps({'passed':len(results),'checks':results},ensure_ascii=False,indent=2))
 print('ALL CHECKS PASSED',len(results),flush=True)
finally:
 stop=True
 for process in reversed(processes):
  if process.poll() is None:
   os.killpg(process.pid,signal.SIGINT)
   try:process.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)

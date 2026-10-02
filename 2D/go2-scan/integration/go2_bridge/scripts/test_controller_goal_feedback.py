"""Isolated real-controller test. No driver, planner, safety gate or motion bridge."""
import os
import signal
import subprocess
import time
from pathlib import Path

root = Path(__file__).resolve().parents[3]
cache = root / '.agent-cache' / 'controller_feedback_test'
cache.mkdir(parents=True, exist_ok=True)
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11379'
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
os.environ['ROS_HOME'] = str(cache)
children = []
logs = []
try:
    for name, args in [('master', ['roscore','-p','11379'])]:
        log = open(cache / (name+'.log'), 'w')
        logs.append(log)
        children.append(subprocess.Popen(args, stdout=log, stderr=log, start_new_session=True))
    import rosgraph
    deadline = time.monotonic()+10
    while not rosgraph.is_master_online():
        if time.monotonic()>deadline: raise RuntimeError('isolated master failed')
        time.sleep(.1)
    import rospy
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int32
    from scan_planner.msg import Bspline
    rospy.init_node('test_goal_execution', disable_signals=True)
    rospy.set_param('/body_pose_topic', '/test/odom')
    params=dict(time_forward=.1,kp_pos=.8,kp_yaw=1.5,
                max_vx=.75,max_vy=.35,max_vyaw=1.,finish_dist=.2)
    for k,v in params.items(): rospy.set_param('/closed_loop_controller/'+k,v)
    log=open(cache/'controller.log','w'); logs.append(log)
    binary=root/'algorithms/local_planning/scan_planner/devel/lib/scan_planner/closed_loop_controller'
    children.append(subprocess.Popen([str(binary)], stdout=log, stderr=log, start_new_session=True))
    odom_pub=rospy.Publisher('/test/odom',Odometry,queue_size=2)
    traj_pub=rospy.Publisher('/planning/bspline',Bspline,queue_size=2)
    stop_pub=rospy.Publisher('/planning/stop_trajectory',Int32,queue_size=2)
    completed=[]; commands=[]
    rospy.Subscriber('/planning/finished_trajectory',Int32,lambda m:completed.append(m.data))
    rospy.Subscriber('/cmd_vel',Twist,lambda m:commands.append((time.monotonic(),m.linear.x)))
    def pump(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            odom=Odometry(); odom.header.stamp=rospy.Time.now(); odom.pose.pose.orientation.w=1
            odom_pub.publish(odom); time.sleep(.02)
    def trajectory(number,x):
        m=Bspline(); m.order=3; m.traj_id=number; m.start_time=rospy.Time.now()
        m.pos_pts=[Point(x,0,0) for _ in range(4)]
        m.knots=[-3.,-2.,-1.,0.,1.,2.,3.,4.]
        traj_pub.publish(m)
    pump(1)
    trajectory(1,0); pump(1.8)
    assert 1 in completed, 'completed trajectory was not acknowledged'
    stop_pub.publish(Int32(1)); pump(.3)
    trajectory(1,1); pump(.4)
    assert max(abs(v) for t,v in commands[-5:])==0, 'stopped trajectory was replayed'
    trajectory(2,1); pump(.4)
    stop_pub.publish(Int32(1)); pump(.4)
    assert any(abs(v)>.1 for t,v in commands[-5:]), 'old stop incorrectly stopped new trajectory'
    stop_pub.publish(Int32(2)); pump(.3)
    assert max(abs(v) for t,v in commands[-5:])==0, 'current trajectory stop failed'
    print('PASS: completion, stale trajectory rejection, stale stop isolation, active stop')
finally:
    for process in reversed(children):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=5)
    for log in logs: log.close()

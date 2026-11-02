from pathlib import Path
import json, math, zipfile, platform, time, os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image as RosImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import pybullet as p
import pybullet_data
import imageio.v2 as imageio
from PIL import Image, ImageDraw

OUT=Path('pai_ros2_colab_output');OUT.mkdir(exist_ok=True)
# 学習: 赤=停止、緑=進行。色のルール閾値ではなく学習したsoftmaxで分類。
rng=np.random.default_rng(7)
centers=np.array([[.9,.06,.06],[.06,.85,.12]])
y=np.repeat(np.arange(2),800)
x=np.clip(centers[y]*rng.uniform(.4,1.1,(1600,1))+rng.normal(0,.04,(1600,3)),0,1)
x=x/(x.sum(1,keepdims=True)+1e-6);x=np.c_[x,np.ones(len(x))]
W=np.zeros((4,2))
for _ in range(1200):
    z=x@W;z-=z.max(1,keepdims=True);prob=np.exp(z);prob/=prob.sum(1,keepdims=True)
    W-=x.T@(prob-np.eye(2)[y])/len(x)
(OUT/'weights.json').write_text(json.dumps(W.tolist()))

def recognize(rgb):
    # 背景除去はルール処理、色の分類は学習モデル。画像全体から画素を抽出。
    a=rgb.astype(float);hi=a.max(2);lo=a.min(2)
    mask=(hi-lo>70)&(hi>65)
    if mask.sum()<20:return 'UNKNOWN',0.,0.,0.
    yy,xx=np.where(mask)
    color=np.median(a[mask],axis=0);color/=color.sum()+1e-6
    z=np.r_[color,1.]@W;z-=z.max();prob=np.exp(z);prob/=prob.sum()
    k=int(prob.argmax())
    return ('STOP','GO')[k],float(prob[k]),float(2*xx.mean()/rgb.shape[1]-1),float((xx.max()-xx.min()+1)/rgb.shape[1])

cid=p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81);p.setTimeStep(1/240)
p.loadURDF('plane.urdf')
robot=p.loadURDF('husky/husky.urdf',[0,0,.18])
# 既存モデルの装飾色が標識検出に混ざらないようグレーに統一。
for link in range(-1,p.getNumJoints(robot)):p.changeVisualShape(robot,link,rgbaColor=[.22,.24,.27,1])
wheels=[]
for j in range(p.getNumJoints(robot)):
    name=p.getJointInfo(robot,j)[1].decode()
    if 'wheel' in name:wheels.append((j,name))
assert len(wheels)==4,wheels
panel_visual=p.createVisualShape(p.GEOM_BOX,halfExtents=[.015,.35,.35],rgbaColor=[.9,.06,.06,1])
# 標識は非接触の信号板。荷扱いや障害物回避は対象外。
sign=p.createMultiBody(baseMass=0,baseVisualShapeIndex=panel_visual,basePosition=[7,0,.65])
projection=p.computeProjectionMatrixFOV(60,320/240,.05,20)
scene_projection=p.computeProjectionMatrixFOV(55,640/480,.1,30)
scene_view=p.computeViewMatrix([8,-8,6],[3,0,.2],[0,0,1])
GOAL_X=4.0
# ゴールは信号とは別の白い床枠。装飾は非衝突モデル。
def marker_box(half, position, color):
    visual=p.createVisualShape(p.GEOM_BOX,halfExtents=half,rgbaColor=color)
    return p.createMultiBody(baseMass=0,baseVisualShapeIndex=visual,basePosition=position)
for xx in (GOAL_X-.5,GOAL_X+.5):
    marker_box([.035,.9,.012],[xx,0,.012],[1,1,1,1])
for yy in (-.9,.9):
    marker_box([.5,.035,.012],[GOAL_X,yy,.012],[1,1,1,1])
marker_box([.025,.9,.015],[GOAL_X,0,.015],[1,1,1,1])
for yy in (-1.0,1.0):
    marker_box([.04,.04,.3],[GOAL_X,yy,.3],[1,1,1,1])

def project_scene(point):
    view=np.array(scene_view).reshape(4,4,order='F')
    proj=np.array(scene_projection).reshape(4,4,order='F')
    v=proj@view@np.r_[point,1.];v=v[:3]/v[3]
    return int((v[0]+1)*320),int((1-v[1])*240)

def signal_color(t):
    return 'STOP' if t<8 or 22<=t<28 else 'GO'


def camera():
    pos,q=p.getBasePositionAndOrientation(robot);R=np.array(p.getMatrixFromQuaternion(q)).reshape(3,3)
    eye=np.array(pos)+R@np.array([.55,0,.45]);target=eye+R@np.array([1,0,0]);up=R@np.array([0,0,1])
    view=p.computeViewMatrix(eye.tolist(),target.tolist(),up.tolist())
    return np.asarray(p.getCameraImage(320,240,view,projection,renderer=p.ER_TINY_RENDERER)[2],dtype=np.uint8).reshape(240,320,4)[:,:,:3]

def motor(v,w):
    for j,name in wheels:
        speed=(v+(-1 if 'left' in name else 1)*.5*w)/.165
        p.setJointMotorControl2(robot,j,p.VELOCITY_CONTROL,targetVelocity=speed,force=60)


# Three real rclpy nodes, one process and one executor. No direct function call
# from simulator to classifier/controller; action is received through /cmd_vel.
class SimulationNode(Node):
    def __init__(self):
        super().__init__('pybullet_simulator')
        self.images=self.create_publisher(RosImage,'/camera/image',10)
        self.odometry=self.create_publisher(Odometry,'/odom',10)
        self.create_subscription(Twist,'/cmd_vel',self.on_cmd,10)
        self.create_subscription(String,'/task/status',self.on_status,10)
        self.command=(0.,0.);self.command_count=0;self.status=None
    def on_cmd(self,m):
        self.command=(m.linear.x,m.angular.z);self.command_count+=1
    def on_status(self,m):self.status=json.loads(m.data)
    def publish(self,rgb,frame,t):
        pos,q=p.getBasePositionAndOrientation(robot)
        o=Odometry();o.header.stamp.sec=frame;o.header.frame_id='odom';o.child_frame_id='base_link'
        o.pose.pose.position.x=float(pos[0]);o.pose.pose.position.y=float(pos[1]);o.pose.pose.position.z=float(pos[2])
        o.pose.pose.orientation.x=float(q[0]);o.pose.pose.orientation.y=float(q[1]);o.pose.pose.orientation.z=float(q[2]);o.pose.pose.orientation.w=float(q[3])
        self.odometry.publish(o)
        im=RosImage();im.header.stamp.sec=frame;im.header.frame_id='camera'
        im.height=rgb.shape[0];im.width=rgb.shape[1];im.encoding='rgb8';im.is_bigendian=0;im.step=im.width*3
        im.data=np.ascontiguousarray(rgb,dtype=np.uint8).tobytes()
        self.images.publish(im)

class VisionNode(Node):
    def __init__(self):
        super().__init__('ai_signal_recognizer')
        self.pub=self.create_publisher(String,'/ai/observation',10)
        self.create_subscription(RosImage,'/camera/image',self.on_image,10)
        self.count=0
    def on_image(self,m):
        a=np.frombuffer(bytes(m.data),dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        label,confidence,cx,width=recognize(a)
        out=String();out.data=json.dumps({'frame':m.header.stamp.sec,'label':label,'confidence':confidence,'cx':cx,'width':width})
        self.pub.publish(out);self.count+=1

class ControllerNode(Node):
    def __init__(self):
        super().__init__('signal_controller')
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.status_pub=self.create_publisher(String,'/task/status',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(String,'/ai/observation',self.on_observation,10)
        self.positions={};self.pending={};self.history=[];self.done=False;self.count=0
    def on_odom(self,m):
        frame=m.header.stamp.sec
        self.positions[frame]=float(m.pose.pose.position.x)
        self.process(frame)
    def on_observation(self,m):
        obs=json.loads(m.data);self.pending[obs['frame']]=obs;self.process(obs['frame'])
    def process(self,frame):
        if frame not in self.positions or frame not in self.pending:return
        x=self.positions.pop(frame);obs=self.pending.pop(frame)
        label=obs['label'];confidence=obs['confidence'];cx=obs['cx']
        self.history.append(label if confidence>.8 else 'UNKNOWN');self.history=self.history[-4:]
        stable=len(self.history)==4 and len(set(self.history))==1
        self.done=self.done or x>=GOAL_X
        v=w=0.;state='WAIT'
        if self.done:state='ARRIVED'
        elif stable and label=='GO':
            v=.105;w=float(np.clip(-.8*cx,-.3,.3));state='DRIVE'
        elif label=='UNKNOWN':state='UNCERTAIN_STOP'
        cmd=Twist();cmd.linear.x=v;cmd.angular.z=w;self.pub.publish(cmd)
        status=String();status.data=json.dumps({**obs,'state':state,'v':v,'w':w,'odom_x':x})
        self.status_pub.publish(status);self.count+=1

assert os.environ.get('ROS_DISTRO')=='humble', 'ROS_DISTRO must be humble'
rclpy.init()
sim_node=SimulationNode();vision_node=VisionNode();control_node=ControllerNode()
executor=SingleThreadedExecutor()
for node in (sim_node,vision_node,control_node):executor.add_node(node)
# Wait for DDS discovery before simulation starts.
limit=time.monotonic()+20
while not (sim_node.images.get_subscription_count() and vision_node.pub.get_subscription_count() and control_node.pub.get_subscription_count() and sim_node.odometry.get_subscription_count()):
    executor.spin_once(timeout_sec=.05)
    if time.monotonic()>limit:raise RuntimeError('ROS2 discovery timeout; simulation has not started')

FPS=10;SECONDS=60;records=[];history=[];done=False;events=[]
video=OUT/'PAI_final_DEMO_ID.mp4'
writer=imageio.get_writer(str(video),fps=FPS,codec='libx264',macro_block_size=16)
try:
    for frame in range(FPS*SECONDS):
        t=frame/FPS
        # 環境シナリオ。制御器は時刻・信号の正解を使わず画像から信号を判定。
        truth=signal_color(t)
        p.changeVisualShape(sign,-1,rgbaColor=([.9,.06,.06,1] if truth=='STOP' else [.06,.85,.12,1]))
        rgb=camera()
        previous_commands=sim_node.command_count
        sim_node.publish(rgb,frame,t)
        deadline=time.monotonic()+5
        while not (sim_node.command_count>previous_commands and sim_node.status is not None and sim_node.status['frame']==frame):
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()>deadline:
                motor(0.,0.)
                raise RuntimeError(f'ROS2 message timeout at frame {frame}; robot stopped. No successful result written.')
        status=sim_node.status
        label=status['label'];confidence=status['confidence'];state=status['state']
        v,w=sim_node.command
        if state=='ARRIVED' and not done:
            done=True;events.append({'event':'arrived','t':t})
        motor(v,w)
        for _ in range(24):p.stepSimulation()
        pos,q=p.getBasePositionAndOrientation(robot)
        records.append({'t':t,'label':label,'confidence':confidence,'state':state,'x':pos[0],'y':pos[1],'v':v,'w':w,'scenario_signal':truth})
        scene=np.asarray(p.getCameraImage(640,480,scene_view,scene_projection,renderer=p.ER_TINY_RENDERER)[2],dtype=np.uint8).reshape(480,640,4)[:,:,:3]
        canvas=Image.new('RGB',(960,480),'#101c30');canvas.paste(Image.fromarray(scene.astype(np.uint8)),(0,0));canvas.paste(Image.fromarray(rgb.astype(np.uint8)),(640,0))
        d=ImageDraw.Draw(canvas)
        gx,gy=project_scene([GOAL_X,0,.1])
        d.rectangle((gx-45,gy-28,gx+45,gy-8),fill='white')
        d.text((gx-35,gy-25),'GOAL  x=4m',fill='black')
        for k,text in enumerate(['PHYSICAL AI | ROS2 HUMBLE',f'Simulation: {t:.1f}s | playback 1x',f'AI: {label}  confidence {confidence:.3f}',f'Controller: {state}',f'x={pos[0]:.2f} m  y={pos[1]:.2f} m','Red: stop / Green: proceed','Red 0-8s,22-28s (scenario)','Wheels driven by motor velocity','ROS2 topics active / No Gazebo']):
            d.text((650,255+k*23),text,fill='white')
        writer.append_data(np.asarray(canvas))
        if frame%100==0 or (frame>0 and records[-1]['state']!=records[-2]['state']):print(f'{t:.0f}s: {state}, x={pos[0]:.2f}',flush=True)
finally:
    writer.close();p.disconnect()
    executor.shutdown()
    for node in (sim_node,vision_node,control_node):node.destroy_node()
    rclpy.shutdown()
result={'simulator':'PyBullet','ros2':True,'ros_distro':os.environ.get('ROS_DISTRO'),'ros_node_names':['pybullet_simulator','ai_signal_recognizer','signal_controller'],'image_messages_processed':vision_node.count,'controller_messages_processed':control_node.count,'cmd_vel_received':sim_node.command_count,'duration_seconds':SECONDS,'playback_speed':1,'arrived':done,'events':events,'start_x':records[0]['x'],'end_x':records[-1]['x'],'python':platform.python_version(),'target_arrival_seconds':50,'commanded_speed_mps':.105,'goal_x':GOAL_X,'goal_position_error_m':records[-1]['x']-GOAL_X,'red_interval_displacement_m':next(r['x'] for r in records if r['t']==28)-next(r['x'] for r in records if r['t']==22),'notes':'AI classifies camera RGB for red/green. Signal changes are scripted environment events. Goal arrival uses simulator x position as an odometry substitute, NOT visual goal recognition. White goal frame and GOAL overlay are display markers.'}
(OUT/'results.json').write_text(json.dumps(result,indent=2))
(OUT/'trajectory.json').write_text(json.dumps(records))
report=f"""# PAI最終課題 レポート原稿（要確認）
アカウントID：DEMO_ID
## 目的
AIが赤・緑の標識を画像から分類し、車輪型ロボットが停止・進行・目的地到達停止を実行する。
## 環境とAI
Google Colab内のRoboStack版ROS2 Humble専用環境、PyBullet物理シミュレータ。合成RGB 1600件でsoftmax分類器を学習。
カメラRGBから彩度で標識候補を抽出し、学習した重みで赤・緑を分類。4回連続判定と信頼度閾値で進行を許可。
## 結果（この実行から自動記入）
目的地到達判定：{done}。位置 x：{records[0]['x']:.3f} → {records[-1]['x']:.3f} m。
これは1回の実行結果であり、成功率ではない。動画とtrajectory.jsonを確認すること。
## 工夫と限界
信号は0〜8秒と22〜28秒に赤、それ以外は緑とする環境側のシナリオ。AIが切替時刻を予測するものではない。
信号はカメラ画像で分類。到着はシミュレータのx座標をオドメトリの代用として判定する。ゴール目印は白い床枠とGOAL表示で、AIがゴールの文字や枠を認識する実装ではない。車輪はモータ速度制御で動かす。
ROS2 Humbleの3ノードを単一プロセス内で動かし、画像→AI判定→速度指令をROS2トピックで受け渡す。Gazeboは使わない。左折・右折、荷扱い、障害物回避、実画像への汎化は対象外。
ロボットモデルはPyBullet付属Huskyを利用。標識は非衝突の可視化モデル。
## 参考
https://qiita.com/N622/items/9d89f77d85d9da0af29e （当初のROS2構成の参考。本版ではコードを流用せず）
https://robostack.github.io/micromamba.html （ROS2 Humbleの導入方法）
https://github.com/bulletphysics/bullet3 （物理計算、レンダラ、付属ロボットモデル。ライセンスは配布元参照）
提出前に本人がコードと結果を確認し、独自の変更・考察を追記。DEMO_IDを置換しPDF化すること。
"""
(OUT/'REPORT_DRAFT.md').write_text(report)
with zipfile.ZipFile('PAI_ros2_colab_results.zip','w',zipfile.ZIP_DEFLATED) as z:
    for f in OUT.iterdir():z.write(f,f.name)
    for name in ('ros2_smoke.py','bootstrap_ros2.sh'):
        if Path(name).exists():z.write(name,name)
    if Path('ros2_main.py').exists():z.write('ros2_main.py','ros2_main.py')
print(json.dumps(result,indent=2));print('Saved:',video,'and PAI_ros2_colab_results.zip')
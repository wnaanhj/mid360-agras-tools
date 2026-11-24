# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 快速显示路径用历史姿态及陀螺近似对齐包预览，不作为地图写入或里程计依据。
"""仅用于屏幕：按设备时间将小段回波转到共同坐标系；绝不送回建图。"""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

# 接收点数组、按包切分的时间、姿态快照及世界绘制选项；按支持的时间范围旋转/变换，证据不足沿用安全预览路径。
def project(points,motion,snap,world):
    """每个约4ms小组取中点旋转；世界系使用短时匀速平移。缺覆盖绝不拼接旧位姿。"""
    packets,imus=motion
    if not packets or len(imus)<2:return None
    packet=np.asarray(packets); stamps=packet[:,0]; counts=packet[:,1].astype(int)
    clock=packet[-1,2]
    if np.any(packet[:,2]!=clock) or counts.sum()!=len(points) or np.any(np.diff(stamps)<=0):return None
    imu=np.asarray(imus)
    # 只取最新的连续同钟段；禁止跨重新上电、时钟切换或缺包做姿态插值。
    cuts=np.flatnonzero((np.diff(imu[:,0])<=0)|(np.diff(imu[:,0])>.025)|(np.diff(imu[:,1])!=0))
    if len(cuts):imu=imu[cuts[-1]+1:]
    if len(imu)<2 or imu[-1,1]!=clock:return None
    latest=stamps[-1]
    anchor=snap.get('pose_stamp') if world else latest
    if anchor is None:return None
    if world and (not snap.get('active') or snap.get('recovering') or snap.get('lost') or
                  snap.get('pose_clock')!=clock or not -.03<=latest-anchor<=.45):return None
    # 匹配积压时旧速度外推会把实时层平移数十厘米。超出12cm预测距离时
    # 沿用最近已配准扫描，不把不可靠外推当成当前精确位置；不影响保存地图。
    if world and np.linalg.norm(snap.get('velocity',np.zeros(3)))*max(0.,latest-anchor)>.12:return None
    t=imu[:,0]+snap.get('preview_offset',0.)
    low,high=min(anchor,stamps[0]),max(anchor,latest)
    # 两个独立UDP端口到达先后不同；端点最多沿用一个约5ms采样，不跨长缺口。
    if low<t[0]-.008 or high>t[-1]+.008:return None
    if high<=low:return None
    nodes=np.r_[low,t[(t>low)&(t<high)],high]
    mid=(nodes[:-1]+nodes[1:])*.5
    w=(imu[:,2:5]-np.asarray(snap.get('bias',(0,0,0))))@np.asarray(snap.get('preview_Rli',np.eye(3))).T
    rates=np.column_stack([np.interp(mid,t,col) for col in w.T])
    steps=Rotation.from_rotvec(rates*np.diff(nodes)[:,None]).as_matrix()
    mats=np.empty((len(nodes),3,3));mats[0]=np.eye(3)
    for i,step in enumerate(steps):mats[i+1]=mats[i]@step
    interpolate=Slerp(nodes,Rotation.from_matrix(mats))
    groups=((stamps-stamps[0])/.004).astype(int)
    ix=np.r_[0,np.flatnonzero(np.diff(groups))+1]; end=np.r_[ix[1:],len(stamps)]
    centers=(stamps[ix]+stamps[end-1])*.5
    anchor_rot=interpolate([anchor]).as_matrix()[0]
    rotate=interpolate(centers).as_matrix()
    anchor_world=np.asarray(snap['rotation']) if world else np.eye(3)
    rotate=anchor_world@anchor_rot.T@rotate
    lever=np.asarray(snap.get('preview_lever',(0,0,0)))
    shift=(anchor_world@lever)-(rotate@lever)
    if world:shift+=np.asarray(snap['pose'])+(centers-anchor)[:,None]*snap.get('velocity',np.zeros(3))
    offsets=np.r_[0,np.cumsum(counts)]
    out=np.empty_like(points)
    for i,(left,right) in enumerate(zip(offsets[ix],offsets[end])):
        out[left:right]=points[left:right]@rotate[i].T.astype('f4')+shift[i].astype('f4')
    return out

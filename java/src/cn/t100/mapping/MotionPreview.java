// SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
// 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
package cn.t100.mapping;

import java.util.Arrays;

/** Device-time motion projection for display only. Quaternions are [w,x,y,z]. */
/** 源码维护说明：监听和在线显示的IMU旋转预览；不改归档点或原生地图位姿，时序不完整交调用方降级。 */
final class MotionPreview {
    private static final double EPS=1e-8;
    /** 静态预览计算不公开实例。 */
    private MotionPreview(){}
    /** 保存最近陀螺数据的有界时间环形缓冲。 */
    static final class Gyros {
        private static final int CAP=1024;
        private final double[] time=new double[CAP], x=new double[CAP], y=new double[CAP], z=new double[CAP];
        private int first,size;
        /** 复位历史索引，放弃上一时间段。 */
        void clear(){first=size=0;}
        /** 倒序或超过25ms缺口先清空；满容量时只淘汰最早样本。 */
        void add(double[] a){
            if(size>0&&(a[0]<=time[index(size-1)]||a[0]-time[index(size-1)]>.025+EPS))clear();
            if(size==CAP){first=(first+1)%CAP;size--;}
            int i=index(size++);time[i]=a[0];x[i]=a[1];y[i]=a[2];z[i]=a[3];
        }
        /** 把逻辑样本号换算为数组物理下标。 */
        private int index(int i){return (first+i)%CAP;}
        /** 读取逻辑下标对应的设备时间。 */
        private double time(int i){return time[index(i)];}
    }
    /** 验证角速度的时间覆盖及 IMU 到雷达的外参，按点时刻补偿到参考时刻；无法可靠补偿返回空。 */
    static float[] project(float[] points,double[] times,Gyros gyro,double[] r,double[] lever,
                           double[] pose,double[] bias,double latest){
        if(points.length==0||points.length/3!=times.length||gyro.size<2||r==null||r.length!=9||lever==null||lever.length!=3)return null;
        if(!finite(r)||!finite(lever)||(bias!=null&&!finite(bias)))return null;
        for(int i=1;i<times.length;i++)if(times[i]<times[i-1])return null;
        if(!Double.isFinite(latest)||latest+EPS<times[times.length-1])return null;
        double anchor=pose==null?latest:pose[23];
        if(!Double.isFinite(anchor))return null;
        if(pose!=null){
            for(int i=0;i<18;i++)if(!Double.isFinite(pose[i]))return null;
            double lag=latest-anchor;
            if(lag<-.03||lag>.45||Math.hypot(Math.hypot(pose[12],pose[13]),pose[14])*Math.max(0,lag)>.12)return null;
        }
        double low=Math.min(anchor,times[0]),high=Math.max(anchor,latest);
        if(high<=low||low<gyro.time(0)-.008-EPS||high>gyro.time(gyro.size-1)+.008+EPS)return null;
        double[] ticks=new double[gyro.size+2],quats=new double[ticks.length*4];int n=0;
        ticks[n++]=low;
        for(int i=0;i<gyro.size;i++){double t=gyro.time(i);if(t>low&&t<high)ticks[n++]=t;}
        ticks[n++]=high;quats[0]=1;
        double bx=bias==null?0:bias[0],by=bias==null?0:bias[1],bz=bias==null?0:bias[2];
        double[] step=new double[4],prev=new double[4],next=new double[4];int pos=0;
        for(int i=1;i<n;i++){
            double dt=ticks[i]-ticks[i-1],mid=(ticks[i]+ticks[i-1])*.5;
            while(pos+2<gyro.size&&gyro.time(pos+1)<mid)pos++;
            int a=gyro.index(pos),b=gyro.index(pos+1);
            double f=Math.max(0,Math.min(1,(mid-gyro.time[a])/(gyro.time[b]-gyro.time[a])));
            double x=gyro.x[a]+f*(gyro.x[b]-gyro.x[a])-bx;
            double y=gyro.y[a]+f*(gyro.y[b]-gyro.y[a])-by;
            double z=gyro.z[a]+f*(gyro.z[b]-gyro.z[a])-bz;
            double u=r[0]*x+r[1]*y+r[2]*z,v=r[3]*x+r[4]*y+r[5]*z,w=r[6]*x+r[7]*y+r[8]*z;
            double norm=Math.sqrt(u*u+v*v+w*w),angle=.5*dt*norm,k=norm<1e-12?.5*dt:Math.sin(angle)/norm;
            step[0]=Math.cos(angle);step[1]=u*k;step[2]=v*k;step[3]=w*k;
            System.arraycopy(quats,4*(i-1),prev,0,4);multiply(prev,step,next);normalize(next);
            System.arraycopy(next,0,quats,4*i,4);
        }
        double[] inverse=new double[4];
        interpolate(anchor,ticks,quats,n,inverse);
        for(int i=1;i<4;i++)inverse[i]=-inverse[i];
        float[] out=new float[points.length];
        double[] q=new double[4],rotation=new double[4],rows=new double[9],transform=new double[9];
        double ax=pose==null?lever[0]:pose[9]+pose[0]*lever[0]+pose[1]*lever[1]+pose[2]*lever[2];
        double ay=pose==null?lever[1]:pose[10]+pose[3]*lever[0]+pose[4]*lever[1]+pose[5]*lever[2];
        double az=pose==null?lever[2]:pose[11]+pose[6]*lever[0]+pose[7]*lever[1]+pose[8]*lever[2];
        for(int i=0;i<times.length;){
            int j=i+1;long bin=(long)((times[i]-times[0])/.004);
            while(j<times.length&&(long)((times[j]-times[0])/.004)==bin)j++;
            double at=(times[i]+times[j-1])*.5;
            interpolate(at,ticks,quats,n,q);multiply(inverse,q,rotation);matrix(rotation,rows);
            double tx=ax,ty=ay,tz=az;
            double[] m=rows;
            if(pose!=null){
                for(int row=0;row<3;row++)for(int col=0;col<3;col++)transform[3*row+col]=
                    pose[3*row]*rows[col]+pose[3*row+1]*rows[col+3]+pose[3*row+2]*rows[col+6];
                m=transform;double d=at-anchor;tx+=d*pose[12];ty+=d*pose[13];tz+=d*pose[14];
            }
            tx-=m[0]*lever[0]+m[1]*lever[1]+m[2]*lever[2];
            ty-=m[3]*lever[0]+m[4]*lever[1]+m[5]*lever[2];
            tz-=m[6]*lever[0]+m[7]*lever[1]+m[8]*lever[2];
            for(int k=i;k<j;k++){
                int off=3*k;double x=points[off],y=points[off+1],z=points[off+2];
                out[off]=(float)(m[0]*x+m[1]*y+m[2]*z+tx);
                out[off+1]=(float)(m[3]*x+m[4]*y+m[5]*z+ty);
                out[off+2]=(float)(m[6]*x+m[7]*y+m[8]*z+tz);
            }
            i=j;
        }
        return out;
    }
    /** 确认矩阵、杆臂或零偏各分量均有限。 */
    private static boolean finite(double[] a){for(double v:a)if(!Double.isFinite(v))return false;return true;}
    /** 按标量在首位的顺序相乘两个四元数。 */
    private static void multiply(double[] a,double[] b,double[] o){
        o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
        o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
        o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
        o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
    }
    /** 将积分四元数归一化，限制数值漂移。 */
    private static void normalize(double[] q){double n=0;for(double x:q)n+=x*x;n=Math.sqrt(n);for(int i=0;i<4;i++)q[i]/=n;}
    /** 在排序时间轴查找准确姿态或两邻点并插值。 */
    private static void interpolate(double at,double[] ticks,double[] a,int n,double[] q){
        int index=Arrays.binarySearch(ticks,0,n,at);
        if(index>=0){System.arraycopy(a,index*4,q,0,4);return;}
        int i=Math.max(0,Math.min(n-2,-index-2)),left=i*4,right=left+4;
        double f=Math.max(0,Math.min(1,(at-ticks[i])/(ticks[i+1]-ticks[i]))),dot=0;
        for(int j=0;j<4;j++)dot+=a[left+j]*a[right+j];
        double sign=dot<0?-1:1;dot=Math.min(1,Math.abs(dot));
        double u=1-f,v=f;
        if(dot<.99999){double angle=Math.acos(dot),den=Math.sin(angle);u=Math.sin((1-f)*angle)/den;v=Math.sin(f*angle)/den;}
        for(int j=0;j<4;j++)q[j]=u*a[left+j]+sign*v*a[right+j];
        normalize(q);
    }
    /** 将标量在首位的单位四元数转成行主序3×3旋转矩阵。 */
    private static void matrix(double[] q,double[] m){
        double w=q[0],x=q[1],y=q[2],z=q[3];
        m[0]=1-2*(y*y+z*z);m[1]=2*(x*y-w*z);m[2]=2*(x*z+w*y);
        m[3]=2*(y*x+w*z);m[4]=1-2*(x*x+z*z);m[5]=2*(y*z-w*x);
        m[6]=2*(z*x-w*y);m[7]=2*(z*y+w*x);m[8]=1-2*(x*x+y*y);
    }
}

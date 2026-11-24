// SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
// 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
package cn.t100.mapping;

import java.util.*;
import java.util.concurrent.CancellationException;

/** Offline tools; coordinates and positive distance parameters are in meters.
 * Statistical value is standard deviations. Selection is true for the affected points.
 * For leveling, selection limits the plane fit, but the rigid transform applies to all points.
 * For crop/delete, a nonempty selection is required. Inferred flags refer to returned points.
 */
/** 源码维护说明：对完整XYZ做选区修复和候选生成；校验掩码、保留推断标记，所有长循环经可取消进度回调。 */
public final class PointTools {
    /** 静态点云工具无共享可变状态。 */
    private PointTools() {}
    /** 任务进度及中止回调。 */
    public interface Progress { /* 接收百分比与当前步骤；返回假可取消操作。 */ boolean update(int percent,String message); }
    /** 包含处理后的点、推断标签和说明。 */
    public static final class Result {
        public final float[] points;
        public final String report;
        public final boolean[] inferred;
        /** 将同一结果的坐标与对应标记绑定。 */
        Result(float[] pts,String msg,boolean[] flags){points=pts;report=msg;inferred=flags;}
    }
    /** 每次报告进度时检查中断与用户取消。 */
    private static void tick(Progress p,int pc,String msg) {
        if(Thread.currentThread().isInterrupted() || (p!=null && !p.update(pc,msg))) throw new CancellationException();
    }
    /** 每1024项映射一次当前阶段进度，避免逐点刷新。 */
    private static void step(Progress p,int i,int n,int start,int end,String msg) {
        if((i&1023)==0) tick(p,start+(int)((long)i*(end-start)/Math.max(1,n)),msg);
    }
    /** 无选区即全图，否则按点掩码决定是否参与。 */
    private static boolean chosen(boolean[] m,int i){return m==null || m[i];}
    /** 用默认值替换无效或非正参数。 */
    private static double param(double a,double d){return Double.isFinite(a) && a>0?a:d;}
    /** 核对XYZ、掩码与有限坐标，长循环可取消。 */
    private static void check(float[] a,boolean[] m,Progress p) {
        if(a==null || a.length%3!=0 || (m!=null && m.length!=a.length/3)) throw new IllegalArgumentException("点云或选区尺寸不匹配");
        for(int i=0;i<a.length;i++) {
            if((i&16383)==0) tick(p,0,"检查坐标");
            if(!Float.isFinite(a[i])) throw new IllegalArgumentException("点云含无效坐标");
        }
    }
    /** 发布最终进度并构造结果。 */
    private static Result finish(float[] a, String msg, boolean[] f,Progress p) {
        tick(p,100,msg); return new Result(a,msg,f);
    }
    /** 按保留掩码压缩坐标和同序标记；全保留时复用原数组。 */
    private static Result filter(float[] a,boolean[] keep,String msg,boolean[] sourceFlags,Progress p) {
        int n=0;
        for(int i=0;i<keep.length;i++){step(p,i,keep.length,90,95,"统计保留点");if(keep[i])n++;}
        if(n==keep.length) return finish(a,msg+"："+n+" 点",sourceFlags,p);
        float[] b=new float[n*3]; boolean[] flags=sourceFlags==null?null:new boolean[n]; int j=0;
        for(int i=0;i<keep.length;i++) {
            step(p,i,keep.length,95,100,"整理点云");
            if(keep[i]){System.arraycopy(a,i*3,b,j,3);if(flags!=null)flags[j/3]=sourceFlags[i];j+=3;}
        }
        return finish(b,msg+"："+keep.length+" → "+n+" 点",flags,p);
    }
    /** 调用不附带推断点标签的兼容工具入口。 */
    public static Result apply(String tool,float[] a,double value,boolean[] mask,Progress p) {
        return apply(tool,a,value,mask,null,p);
    }
    /** Carries existing inferred flags through filters and coordinate transforms. */
    /** 先检验标签与选区，再分派到裁剪、去噪、平面或补点。 */
    public static Result apply(String tool,float[] a,double value,boolean[] mask,boolean[] sourceFlags,Progress p) {
        if(sourceFlags!=null&&(a==null||sourceFlags.length!=a.length/3))throw new IllegalArgumentException("推断点标记尺寸不匹配");
        check(a,mask,p);
        int n=a.length/3;
        switch(tool){
            case "crop": case "delete": {
                if(mask==null) throw new IllegalArgumentException("请先圈选点");
                boolean any=false;boolean[] keep=new boolean[n];
                for(int i=0;i<n;i++){step(p,i,n,0,90,"读取选区"); any|=mask[i]; keep[i]=tool.equals("crop")?mask[i]:!mask[i];}
                if(!any) throw new IllegalArgumentException("选区没有点");
                return filter(a,keep,tool.equals("crop")?"裁剪":"手动删除",sourceFlags,p);
            }
            case "radius": return radius(a,param(value,.20),mask,sourceFlags,p);
            case "statistical": return statistical(a,param(value,2),mask,sourceFlags,p);
            case "fragments": return fragments(a,param(value,.25),mask,sourceFlags,p);
            case "ground": case "level": case "flatten": case "plane":
                return planeTool(tool,a,param(value,.18),mask,sourceFlags,p);
            case "fill": return fill(a,param(value,.25),mask,sourceFlags,p);
            case "dynamic": case "temporal":
                throw new IllegalArgumentException("需要逐帧的时间、位姿和点，合并点云无法验证动态或分时重影");
            default: throw new IllegalArgumentException("未知点云工具："+tool);
        }
    }
    /** A complete time-stamped archive observation, transformed by the caller into the map frame. */
    /** 某一时刻相同世界坐标系的观察点。 */
    public static final class TimedFrame {
        public final double seconds;
        public final float[] mapPoints;
        /** 保存相对时间和配准后的观测。 */
        public TimedFrame(double seconds,float[] mapPoints){this.seconds=seconds;this.mapPoints=mapPoints;}
    }
    /** 时域候选掩码和证据说明。 */
    public static final class TemporalResult {
        public final boolean[] candidates;
        public final String report;
        /** 记录识别结果，不修改地图。 */
        TemporalResult(boolean[] mask,String message){candidates=mask;report=message;}
    }
    /**
     * Marks only points seen in one brief period and with nearby but nonmatching returns in
     * two independent periods. Proximity is a coverage proxy, so review before any removal.
     * Frames must be complete, chronologically ordered, and in the same map frame.
     */
    /** 比较分时观测的覆盖与重复支持，辨认动态候选。 */
    public static TemporalResult dynamicCandidates(float[] map,boolean[] selection,TimedFrame[] frames,
            double matchMeters,double coverageMeters,double minGapSeconds,Progress p) {
        check(map,selection,p);
        if(frames==null||frames.length<3||frames.length>24||!Double.isFinite(matchMeters)||matchMeters<=0
                ||!Double.isFinite(coverageMeters)||coverageMeters<=matchMeters
                ||!Double.isFinite(minGapSeconds)||minGapSeconds<=0)
            throw new IllegalArgumentException("需要 3–24 个完整分时帧、有效匹配/覆盖半径和时段间隔");
        final int n=map.length/3;int[] hits=new int[n],cover=new int[n];
        double prior=Double.NEGATIVE_INFINITY,lastPeriod=prior;int periods=0;
        for(TimedFrame frame:frames){
            if(frame==null||!Double.isFinite(frame.seconds)||frame.seconds<=prior
                    ||frame.mapPoints==null||frame.mapPoints.length<3||frame.mapPoints.length%3!=0)
                throw new IllegalArgumentException("分时观测须按时间顺序提供，且每帧有完整的地图坐标点");
            prior=frame.seconds;
            if(frame.seconds-lastPeriod>=minGapSeconds){lastPeriod=frame.seconds;periods++;}
        }
        if(periods<3)throw new IllegalArgumentException("至少需三个时间分离的观测时段");
        for(int j=0;j<frames.length;j++){
            final int start=j*90/frames.length,end=(j+1)*90/frames.length,half=start+(end-start)/2;
            Progress gp=p==null?null:(pc,msg)->p.update(start+pc*(half-start)/100,msg);
            float[] obs=frames[j].mapPoints;
            check(obs,null,gp);
            Grid g=new Grid(obs,coverageMeters,false,null,true,gp,100);
            int bit=1<<j;
            double matched=matchMeters*matchMeters,near=coverageMeters*coverageMeters;
            for(int i=0;i<n;i++){
                step(p,i,n,half,end,"比较分时空间观测");
                if(!chosen(selection,i))continue;
                int k=3*i;
                double x=map[k],y=map[k+1],z=map[k+2];
                int ix=g.x(x),iy=g.y(y),iz=g.z(z);
                if(ix < -LIM-1 || ix>LIM || iy < -LIM-1 || iy>LIM || iz < -LIM-1 || iz>LIM)continue;
                boolean seen=false,close=false;
                scan:for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)for(int dz=-1;dz<=1;dz++){
                    Cell c=g.get(ix+dx,iy+dy,iz+dz);
                    if(c==null)continue;
                    for(int k2=c.head,scanned=0;k2>=0;k2=g.next[k2]){
                        if((++scanned&16383)==0)tick(p,half+(int)((long)i*(end-half)/Math.max(1,n)),"比较分时空间观测");
                        double d=sq(x-obs[k2*3])+sq(y-obs[k2*3+1])+sq(z-obs[k2*3+2]);
                        if(d<=matched){seen=true;break scan;}
                        if(d<=near)close=true;
                    }
                }
                if(seen)hits[i]|=bit;else if(close)cover[i]|=bit;
            }
        }
        boolean[] candidate=new boolean[n];int count=0;
        for(int i=0;i<n;i++){
            step(p,i,n,90,100,"标记动态候选");
            if(hits[i]==0||Integer.bitCount(cover[i])<2)continue;
            double first=frames[Integer.numberOfTrailingZeros(hits[i])].seconds;
            double last=frames[31-Integer.numberOfLeadingZeros(hits[i])].seconds;
            if(last-first>=minGapSeconds)continue;
            double other=Double.NEGATIVE_INFINITY;
            for(int bits=cover[i];bits!=0;bits&=bits-1){
                double stamp=frames[Integer.numberOfTrailingZeros(bits)].seconds;
                if(first-stamp<minGapSeconds&&stamp-last<minGapSeconds)continue;
                if(stamp-other>=minGapSeconds){
                    if(Double.isFinite(other)){candidate[i]=true;count++;break;}
                    other=stamp;
                }
            }
        }
        String report="动态候选："+count+" 点，依据 "+frames.length+" 个真实分时观测。邻近回波只作覆盖参考，需人工核对。";
        tick(p,100,report);return new TemporalResult(candidate,report);
    }
    // Signed 21-bit voxel coordinates, with the origin at the first cloud point.
    private static final int LIM=1<<20;
    /** 打包三轴各21位的有界网格索引，越界用哨兵。 */
    private static long key(int x,int y,int z){
        if(x < -LIM || x>=LIM || y < -LIM || y>=LIM || z < -LIM || z>=LIM) return Long.MIN_VALUE;
        return ((long)(x+LIM)<<42)|((long)(y+LIM)<<21)|(z+LIM);
    }
    /** 空间网格内部的一格点统计或链索引。 */
    private static final class Cell {
        final int x,y,z;
        int head=-1,count,group=-1;
        double sx,sy,sz;
        float low=Float.POSITIVE_INFINITY,high=Float.NEGATIVE_INFINITY;
        /** 保存格坐标供邻域及聚类遍历。 */
        Cell(int x,int y,int z){this.x=x;this.y=y;this.z=z;}
    }
    /** 体素空间索引，按需持有逐点链或格内统计。 */
    private static final class Grid {
        final double d,ox,oy,oz;
        final HashMap<Long,Cell> map=new HashMap<>();
        final int[] next;
        /** 按工作尺度建立网格点统计、质心及可选逐点链。 */
        Grid(float[] a,double d,boolean flat,boolean[] m,boolean links,Progress p,int end) {
            this.d=d;ox=a.length>0?a[0]:0;oy=a.length>0?a[1]:0;oz=a.length>0?a[2]:0;
            int n=a.length/3;next=links?new int[n]:null;
            for(int i=0;i<n;i++) {
                step(p,i,n,0,end,"建立空间索引"); if(!chosen(m,i))continue;
                int j=i*3,x=x(a[j]),y=y(a[j+1]),z=flat?0:z(a[j+2]);long k=key(x,y,z);
                if(k==Long.MIN_VALUE) throw new IllegalArgumentException("坐标跨度过大，请增大空间网格");
                Cell c=map.get(k);
                if(c==null){c=new Cell(x,y,z);map.put(k,c);}
                if(links){next[i]=c.head;c.head=i;}
                c.count++;c.sx+=a[j];c.sy+=a[j+1];c.sz+=a[j+2];c.low=Math.min(c.low,a[j+2]);c.high=Math.max(c.high,a[j+2]);
            }
        }
        /** 以首点坐标为原点计算覆盖横轴网格编号。 */
        int x(double x){return (int)Math.floor((x-ox)/d);}
        /** 以首点坐标为原点计算第二轴网格编号。 */
        int y(double y){return (int)Math.floor((y-oy)/d);}
        /** 以首点坐标为原点计算高度网格编号。 */
        int z(double z){return (int)Math.floor((z-oz)/d);}
        /** 从三轴有界格索引取现有格，无数据时为空。 */
        Cell get(int x,int y,int z){return map.get(key(x,y,z));}
    }
    /** 计算距离平方用的标量辅助函数。 */
    private static double sq(double d){return d*d;}
    /** 按真实半径附近点数排除孤立点，选区外原样保留。 */
    private static Result radius(float[] a,double r,boolean[] m,boolean[] sourceFlags,Progress p) {
        int n=a.length/3; Grid g=new Grid(a,r,false,null,true,p,35);
        boolean[] keep=new boolean[n];double rr=r*r;
        for(int i=0;i<n;i++){
            step(p,i,n,35,90,"半径邻域计数");
            if(!chosen(m,i)){keep[i]=true;continue;}
            double x=a[3*i],y=a[3*i+1],z=a[3*i+2]; int ix=g.x(x),iy=g.y(y),iz=g.z(z),hits=0;
            scan:for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)for(int dz=-1;dz<=1;dz++){
                Cell c=g.get(ix+dx,iy+dy,iz+dz);if(c==null)continue;
                double cx=g.ox+c.x*r,cy=g.oy+c.y*r,cz=g.oz+c.z*r;
                if(sq(Math.max(0,Math.max(cx-x,x-cx-r)))+sq(Math.max(0,Math.max(cy-y,y-cy-r)))+sq(Math.max(0,Math.max(cz-z,z-cz-r)))>rr)continue;
                for(int j=c.head,visited=0;j>=0;j=g.next[j]){
                    if((++visited&16383)==0)tick(p,35+(int)(55L*i/Math.max(1,n)),"半径邻域计数");
                    if(j!=i && sq(x-a[3*j])+sq(y-a[3*j+1])+sq(z-a[3*j+2])<=rr && ++hits>=2)break scan;
                }
            }
            keep[i]=hits>=2;
        }
        return filter(a,keep,"半径去噪",sourceFlags,p);
    }
    /** 用有界局部体素质心近似统计邻近密度，避免全图两两比较。 */
    private static Result statistical(float[] a,double sigma,boolean[] m,boolean[] sourceFlags,Progress p) {
        int n=a.length/3;if(n<4)return finish(a,"点数不足，未修改",sourceFlags,p);
        // Bounded local density estimate: nearest voxel centroids, weighted by occupancy.
        Grid g=new Grid(a,.20,false,null,false,p,30);float[] score=new float[n];
        double mean=0,m2=0;int seen=0;
        final int cap=8;double[] nearest=new double[cap];
        for(int i=0;i<n;i++){
            step(p,i,n,30,85,"统计局部点密度");
            if(!chosen(m,i))continue;
            Arrays.fill(nearest,Double.POSITIVE_INFINITY);
            double x=a[3*i],y=a[3*i+1],z=a[3*i+2];int ix=g.x(x),iy=g.y(y),iz=g.z(z);
            for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)for(int dz=-1;dz<=1;dz++){
                Cell c=g.get(ix+dx,iy+dy,iz+dz);if(c==null)continue;
                boolean self=dx==0 && dy==0 && dz==0;
                int count=c.count-(self?1:0);if(count==0)continue;
                double dist=Math.sqrt(sq(x-(c.sx-(self?x:0))/count)+sq(y-(c.sy-(self?y:0))/count)+sq(z-(c.sz-(self?z:0))/count));
                // Same-voxel centroids can coincide by symmetry; retain a local resolution floor.
                if(self)dist=Math.max(dist,.10/Math.cbrt(count));
                for(int k=0;k<cap && count>0;k++)if(dist<nearest[k]){
                    System.arraycopy(nearest,k,nearest,k+1,cap-k-1);nearest[k]=dist;count--;
                }
            }
            double s=0;for(double v:nearest)s+=Double.isFinite(v)?v:.65;
            score[i]=(float)(s/cap);
            double delta=score[i]-mean;mean+=delta/++seen;m2+=delta*(score[i]-mean);
        }
        double threshold=mean+sigma*Math.sqrt(m2/Math.max(1,seen-1));
        boolean[] keep=new boolean[n];
        for(int i=0;i<n;i++){step(p,i,n,85,90,"应用统计阈值");keep[i]=!chosen(m,i)||score[i]<=threshold+1e-7;}
        return filter(a,keep,"统计去噪（体素近邻）",sourceFlags,p);
    }
    /** 遍历占用格连通成分，按规模判定分离碎片。 */
    private static Result fragments(float[] a,double voxel,boolean[] m,boolean[] sourceFlags,Progress p) {
        int n=a.length/3;Grid g=new Grid(a,voxel,false,null,false,p,35);
        ArrayDeque<Cell> q=new ArrayDeque<>();ArrayList<Integer> size=new ArrayList<>();
        int visited=0,max=0;
        for(Cell start:g.map.values()){
            if(start.group>=0)continue;
            int group=size.size();start.group=group;q.add(start);int count=0;
            while(!q.isEmpty()){
                step(p,visited++,g.map.size(),35,75,"合并邻接体素");
                Cell c=q.remove(); count+=c.count;
                for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)for(int dz=-1;dz<=1;dz++){
                    Cell v=g.get(c.x+dx,c.y+dy,c.z+dz);
                    if(v!=null && v.group<0){v.group=group;q.add(v);}
                }
            }
            size.add(count);max=Math.max(max,count);
        }
        int min=Math.min(max,Math.max(6,Math.min(128,(int)Math.ceil(max*.01))));
        boolean[] keep=new boolean[n];
        for(int i=0;i<n;i++){
            step(p,i,n,75,90,"标记独立碎片");
            keep[i]=!chosen(m,i)||size.get(g.get(g.x(a[i*3]),g.y(a[i*3+1]),g.z(a[i*3+2])).group)>=min;
        }
        return filter(a,keep,"碎片清理（体素连通）",sourceFlags,p);
    }
    /** 储存拟合平面的参考点和单位法线。 */
    private static final class Fit {
        final double x,y,z,nx,ny,nz;
        /** 绑定平面通过点及法向三个分量。 */
        Fit(double x,double y,double z,double nx,double ny,double nz){this.x=x;this.y=y;this.z=z;this.nx=nx;this.ny=ny;this.nz=nz;}
        /** 计算点沿单位法线的有符号平面距离。 */
        double dist(double x,double y,double z){return (x-this.x)*nx+(y-this.y)*ny+(z-this.z)*nz;}
    }
    /** 在线累计质心和协方差，供局部平面拟合。 */
    private static final class Cov {
        int count;
        double x,y,z;
        final double[][] c=new double[3][3];
        /** 逐点更新均值及对称二阶量，避免重复扫描。 */
        void add(double a,double b,double d){
            double dx=a-x,dy=b-y,dz=d-z;
            ++count;x+=dx/count;y+=dy/count;z+=dz/count;
            c[0][0]+=dx*(a-x);c[0][1]+=dx*(b-y);c[0][2]+=dx*(d-z);
            c[1][1]+=dy*(b-y);c[1][2]+=dy*(d-z);c[2][2]+=dz*(d-z);
        }
        /** 用协方差最小方向估计法向，点不足或退化时不返回平面。 */
        Fit fit(){
            if(count<3)return null;
            c[1][0]=c[0][1];c[2][0]=c[0][2];c[2][1]=c[1][2];
            double[][] v={{1,0,0},{0,1,0},{0,0,1}};
            for(int i=0;i<24;i++){
                int a=0,b=1;
                if(Math.abs(c[0][2])>Math.abs(c[a][b])){a=0;b=2;}
                if(Math.abs(c[1][2])>Math.abs(c[a][b])){a=1;b=2;}
                if(Math.abs(c[a][b])<=1e-12)break;
                double phi=.5*Math.atan2(2*c[a][b],c[b][b]-c[a][a]);
                double sn=Math.sin(phi),cs=Math.cos(phi),aa=c[a][a],bb=c[b][b],ab=c[a][b];
                for(int k=0;k<3;k++)if(k!=a&&k!=b){
                    double ka=c[k][a],kb=c[k][b];
                    c[k][a]=c[a][k]=cs*ka-sn*kb;c[k][b]=c[b][k]=sn*ka+cs*kb;
                }
                c[a][a]=cs*cs*aa-2*cs*sn*ab+sn*sn*bb;
                c[b][b]=sn*sn*aa+2*cs*sn*ab+cs*cs*bb;c[a][b]=c[b][a]=0;
                for(int k=0;k<3;k++){
                    double ka=v[k][a],kb=v[k][b];v[k][a]=cs*ka-sn*kb;v[k][b]=sn*ka+cs*kb;
                }
            }
            int a=0;if(c[1][1]<c[a][a])a=1;if(c[2][2]<c[a][a])a=2;
            double middle=Math.min(c[(a+1)%3][(a+1)%3],c[(a+2)%3][(a+2)%3]);
            if(middle<=1e-9 || c[a][a]>middle*.25)return null;
            double sign=v[2][a]<0?-1:1;
            return new Fit(x,y,z,sign*v[0][a],sign*v[1][a],sign*v[2][a]);
        }
    }
    /** 按是否寻找地面选择直接拟合选区或筛选地面候选。 */
    private static Fit plane(float[] a,boolean[] m,double tolerance,boolean ground,Progress p) {
        if(!ground){
            Cov cov=new Cov();
            for(int i=0;i<a.length/3;i++){step(p,i,a.length/3,0,62,"拟合选区平面");if(chosen(m,i))cov.add(a[i*3],a[i*3+1],a[i*3+2]);}
            return cov.fit();
        }
        Grid g=new Grid(a,Math.max(.45,3*tolerance),true,m,false,p,22);
        if(g.map.size()<3)return null;
        // Lower envelope, one sample per occupied XY cell. Fit a plane robustly before refining.
        final int bound=3072;ArrayList<Cell> sample=new ArrayList<>(Math.min(bound,g.map.size()));
        Random random=new Random(100);int seen=0;
        for(Cell c:g.map.values()){
            step(p,seen,g.map.size(),22,32,"采样低位地面");seen++;
            if(sample.size()<bound)sample.add(c);else {int j=random.nextInt(seen);if(j<bound)sample.set(j,c);}
        }
        Fit best=null;double bestScore=0;
        for(int i=0;i<128;i++){
            if((i&7)==0)tick(p,32+i*20/128,"寻找地面主平面");
            Cell h=sample.get(random.nextInt(sample.size())),j=sample.get(random.nextInt(sample.size())),k=sample.get(random.nextInt(sample.size()));
            double x=h.sx/h.count,y=h.sy/h.count,z=h.low;
            double ax=j.sx/j.count-x,ay=j.sy/j.count-y,az=j.low-z;
            double bx=k.sx/k.count-x,by=k.sy/k.count-y,bz=k.low-z;
            double nx=ay*bz-az*by,ny=az*bx-ax*bz,nz=ax*by-ay*bx, len=Math.sqrt(nx*nx+ny*ny+nz*nz);
            if(len<1e-8 || Math.abs(nz/len)<.76)continue;
            double s=nz<0?-len:len;Fit f=new Fit(x,y,z,nx/s,ny/s,nz/s);double score=0;
            for(Cell c:sample){
                double d=f.dist(c.sx/c.count,c.sy/c.count,c.low);
                if(Math.abs(d)<=tolerance)score+=1-.4*Math.abs(d)/tolerance;
                else if(d<0)score-=1.4;
            }
            if(score>bestScore){bestScore=score;best=f;}
        }
        if(best==null || bestScore<Math.max(2,sample.size()*.18))return null;
        Cov cov=new Cov();int count=0;
        for(Cell c:g.map.values()){
            step(p,count++,g.map.size(),52,64,"细化地面平面");
            double x=c.sx/c.count,y=c.sy/c.count;
            if(Math.abs(best.dist(x,y,c.low))<=tolerance)cov.add(x,y,c.low);
        }
        Fit refined=cov.fit();
        return refined!=null && refined.nz>=.76?refined:best;
    }
    /** 应用指定平面工具；平整要求选区，无法稳定拟合时不改点。 */
    private static Result planeTool(String op,float[] a,double tol,boolean[] m,boolean[] sourceFlags,Progress p) {
        if(op.equals("flatten")&&m==null)throw new IllegalArgumentException("请先圈选要平整的局部地面");
        Fit f=plane(a,m,tol,!op.equals("plane"),p);
        if(f==null)throw new IllegalArgumentException("未找到足够稳定的平面，请调整选区或容差");
        int n=a.length/3;
        if(op.equals("ground")){
            boolean[] keep=new boolean[n];
            for(int i=0;i<n;i++){step(p,i,n,64,90,"提取地面点");keep[i]=!chosen(m,i)||Math.abs(f.dist(a[i*3],a[i*3+1],a[i*3+2]))<=tol;}
            return filter(a,keep,"地面提取",sourceFlags,p);
        }
        float[] b=a.clone();int moved=0;
        double vx=f.ny,vy=-f.nx,den=1+f.nz;
        for(int i=0;i<n;i++){
            step(p,i,n,64,100,"更新点坐标");int j=3*i;
            double x=a[j],y=a[j+1],z=a[j+2];
            if(op.equals("level")){
                double dx=x-f.x,dy=y-f.y,dz=z-f.z,dot=vx*dx+vy*dy,q=vx*vx+vy*vy;
                b[j]=(float)(x+vy*dz+(vx*dot-q*dx)/den);
                b[j+1]=(float)(y-vx*dz+(vy*dot-q*dy)/den);
                b[j+2]=(float)(z+vx*dy-vy*dx-q*dz/den);
                moved++;
            }else if(chosen(m,i)){
                double d=f.dist(x,y,z);
                if(op.equals("flatten")){
                    if(Math.abs(d)>tol)continue;
                    b[j+2]=(float)(z-d/f.nz);
                }else{
                    b[j]=(float)(x-d*f.nx);b[j+1]=(float)(y-d*f.ny);b[j+2]=(float)(z-d*f.nz);
                }
                moved++;
            }
        }
        return finish(b,(op.equals("level")?"整体调平":op.equals("flatten")?"局部地面平整":"平面投影")+"："+moved+" 点",sourceFlags,p);
    }
    /** 有真实邻接点支撑才生成局部孔洞补点，附推断标签。 */
    private static Result fill(float[] a,double spacing,boolean[] m,boolean[] sourceFlags,Progress p) {
        Grid selected=new Grid(a,spacing,true,m,false,p,25);
        Grid all=m==null?selected:new Grid(a,spacing,true,null,false,
            p==null?null:(pc,msg)->p.update(25+pc,msg),20);
        ArrayList<float[]> fill=new ArrayList<>();
        int count=0, max=Math.min(100_000,Math.max(1,a.length/12));
        double zTolerance=spacing*.55;
        // Only inspect the empty cell to the right of its required left neighbor.
        for(Cell l:selected.map.values()) {
            step(p,count++,selected.map.size(),45,90,"寻找有边界支撑的小孔");
            int x=l.x+1,y=l.y;
            if(all.get(x,y,0)!=null)continue;
            Cell r=selected.get(x+1,y,0),u=selected.get(x,y+1,0),d=selected.get(x,y-1,0);
            if(r==null||u==null||d==null)continue;
            double zMin=Math.min(Math.min(l.low,r.low),Math.min(u.low,d.low));
            double zMax=Math.max(Math.max(l.high,r.high),Math.max(u.high,d.high));
            if(zMax-zMin>zTolerance)continue;
            fill.add(new float[]{(float)(selected.ox+(x+.5)*spacing),(float)(selected.oy+(y+.5)*spacing),
                  (float)((l.sz/l.count+r.sz/r.count+u.sz/u.count+d.sz/d.count)/4)});
            if(fill.size()>=max)break;
        }
        if(fill.isEmpty())return finish(a,"没有找到四周都有支撑的小孔，未修改",sourceFlags,p);
        float[] out=Arrays.copyOf(a,a.length+fill.size()*3);boolean[] inferred=sourceFlags==null?new boolean[out.length/3]:Arrays.copyOf(sourceFlags,out.length/3);
        for(int i=0;i<fill.size();i++){
            step(p,i,fill.size(),90,100,"写入推断点");
            System.arraycopy(fill.get(i),0,out,a.length+i*3,3);
            inferred[a.length/3+i]=true;
        }
        return finish(out,"小孔补点："+fill.size()+" 个推断点",inferred,p);
    }
}




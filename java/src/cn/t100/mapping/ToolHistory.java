// SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
// 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
package cn.t100.mapping;

import java.io.*;
import java.nio.file.*;
import java.util.ArrayList;

/** Disk-backed undo/redo for interactive previews. The caller owns the working arrays. */
/** 源码维护说明：点云编辑的磁盘撤销重做；快照保存XYZ与推断标记，源地图不参与历史覆盖。 */
public final class ToolHistory {
    /** 一个完整编辑版本的点和推断标签。 */
    public static final class Snapshot {
        public final float[] points;
        public final boolean[] inferred;
        /** 绑定同一次读取的对应数组。 */
        Snapshot(float[] pts,boolean[] marks){points=pts;inferred=marks;}
    }
    private static final int MAGIC=0x54313030;
    private static final int MAX_STATES=8;
    private final File dir;
    private final ArrayList<File> states=new ArrayList<>();
    private int cursor=-1;
    /** 指定只用于编辑历史的工作目录。 */
    public ToolHistory(File dir){this.dir=dir;}
    /** 检查当前位置是否有上一个快照。 */
    public synchronized boolean canUndo(){return cursor>0;}
    /** 检查当前位置是否有后续快照。 */
    public synchronized boolean canRedo(){return cursor+1<states.size();}
    /** 清理已登记的历史文件并重置游标。 */
    public synchronized void reset(){
        for(File f:states)f.delete();states.clear();cursor=-1;
    }
    /** 校验数组和标记后独占落盘，再更新历史及重做支路。 */
    public synchronized void record(float[] a,boolean[] flags) throws IOException {
        if(a==null||a.length%3!=0||(flags!=null&&flags.length!=a.length/3))throw new IOException("历史点云或标记长度错误");
        if(!dir.isDirectory()&&!dir.mkdirs())throw new IOException("不能建立修复历史目录");
        File f=File.createTempFile("state_",".bin",dir);
        boolean complete=false;
        try{
            try(DataOutputStream out=new DataOutputStream(new BufferedOutputStream(new FileOutputStream(f),64*1024))){
                out.writeInt(MAGIC);out.writeInt(a.length);out.writeBoolean(flags!=null);
                for(float v:a)out.writeFloat(v);
                if(flags!=null)for(boolean b:flags)out.writeBoolean(b);
            }
            complete=true;
        }finally{if(!complete)f.delete();}
        while(states.size()>cursor+1)states.remove(states.size()-1).delete();
        states.add(f);cursor++;
        // History is disk-backed; retain entries until explicit reset.
    }
    /** 若可撤销，读取前一完整状态。 */
    public synchronized Snapshot undo() throws IOException {return canUndo()?move(cursor-1):null;}
    /** 若可重做，读取后一完整状态。 */
    public synchronized Snapshot redo() throws IOException {return canRedo()?move(cursor+1):null;}
    /** 校验快照格式，完整读出坐标标记后才切换游标。 */
    private Snapshot move(int i) throws IOException {
        File f=states.get(i);
        try(DataInputStream in=new DataInputStream(new BufferedInputStream(new FileInputStream(f),64*1024))){
            if(in.readInt()!=MAGIC)throw new IOException("修复历史格式错误");
            int n=in.readInt();boolean has=in.readBoolean();int count=n/3;
            if(n<0||n%3!=0||f.length()!=9L+n*4L+(has?count:0))throw new IOException("修复历史文件损坏");
            float[] points=new float[n];
            for(int j=0;j<n;j++)points[j]=in.readFloat();
            boolean[] flags=has?new boolean[count]:null;
            if(has)for(int j=0;j<count;j++)flags[j]=in.readBoolean();
            cursor=i; return new Snapshot(points,flags);
        }
    }
}

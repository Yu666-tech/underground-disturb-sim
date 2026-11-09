#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <numeric>
#include <cstring>
struct V {double x,y,z;V operator+(V b)const{return{x+b.x,y+b.y,z+b.z};}V operator-(V b)const{return{x-b.x,y-b.y,z-b.z};}V operator*(double a)const{return{x*a,y*a,z*a};}};
double dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
V cross(V a,V b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double clip(double a){return std::max(0.,std::min(1.,a));}
V read(const double*p){return{p[0],p[1],p[2]};}
static double nearest_impl(const double* triangles,int count,const double* av,const double* bv,double* output,double* squared=nullptr,int* selected=nullptr){
 V a=read(av),b=read(bv),u=b-a;double aa=dot(u,u),best=std::numeric_limits<double>::infinity();V bestpoint{},bestnormal{};
 int current=0,bestindex=-1;
 auto offer=[&](V q,V p,V n){double d=dot(q-p,q-p);if(d<best){best=d;bestpoint=q;bestnormal=n;bestindex=current;}};
 for(int i=0;i<count;++i){current=i;const double* data=triangles+i*9;V v[3]={read(data),read(data+3),read(data+6)};V e1=v[1]-v[0],e2=v[2]-v[0],n=cross(e1,e2);double nn=dot(n,n);
  if(nn<1e-24)continue;
  auto inside=[&](V p){V w=p-v[0];double d00=dot(e1,e1),d01=dot(e1,e2),d11=dot(e2,e2),d20=dot(w,e1),d21=dot(w,e2);double den=d00*d11-d01*d01;if(den<=1e-24)return false;double s=(d11*d20-d01*d21)/den,t=(d00*d21-d01*d20)/den;return s>=-1e-12&&t>=-1e-12&&s+t<=1+1e-12;};
  double denom=dot(n,u);
  if(std::abs(denom)>1e-18){double t=dot(n,v[0]-a)/denom;if(t>=0&&t<=1){V p=a+u*t;if(inside(p)){offer(p,p,n);continue;}}}
  for(V endpoint:{a,b}){V q=endpoint-n*(dot(endpoint-v[0],n)/nn);if(inside(q))offer(q,endpoint,n);}
  for(int j=0;j<3;++j){V c=v[j],d=v[(j+1)%3],edge=d-c,w=a-c;double cc=dot(edge,edge);if(cc<1e-24)continue;double bb=dot(u,edge),dd=dot(u,w),ee=dot(edge,w),den=aa*cc-bb*bb;
   if(den>1e-20){double s=(bb*ee-cc*dd)/den,t=(aa*ee-bb*dd)/den;if(s>=0&&s<=1&&t>=0&&t<=1)offer(c+edge*t,a+u*s,n);}
   for(V endpoint:{a,b})offer(c+edge*clip(dot(endpoint-c,edge)/cc),endpoint,n);
   for(V endpoint:{c,d})offer(endpoint,a+u*clip(dot(endpoint-a,u)/aa),n);
  }
 }
 if(squared)*squared=best;
 if(selected)*selected=bestindex;
 double norm=std::sqrt(dot(bestnormal,bestnormal));output[0]=bestpoint.x;output[1]=bestpoint.y;output[2]=bestpoint.z;output[3]=norm?bestnormal.x/norm:0;output[4]=norm?bestnormal.y/norm:0;output[5]=norm?bestnormal.z/norm:0;return std::sqrt(best);
}
extern "C" double nearest_surface(const double*t,int n,const double*a,const double*b,double*out){return nearest_impl(t,n,a,b,out);}

// Static meshes: exact branch-and-bound; triangle math and tie order stay unchanged.
struct Bounds {double lo[3],hi[3];};
struct BvhNode {Bounds box;int begin,end,left=-1,right=-1;};
class SurfaceBvh {
 std::vector<double> data;
 std::vector<int> order;
 std::vector<Bounds> triangles;
 std::vector<BvhNode> nodes;
 int build(int begin,int end){
  BvhNode node;node.begin=begin;node.end=end;
  for(int k=0;k<3;++k){node.box.lo[k]=std::numeric_limits<double>::infinity();node.box.hi[k]=-node.box.lo[k];}
  for(int i=begin;i<end;++i)for(int k=0;k<3;++k){node.box.lo[k]=std::min(node.box.lo[k],triangles[order[i]].lo[k]);node.box.hi[k]=std::max(node.box.hi[k],triangles[order[i]].hi[k]);}
  int index=static_cast<int>(nodes.size());nodes.push_back(node);
  if(end-begin<=8){std::sort(order.begin()+begin,order.begin()+end);return index;}
  int axis=0;for(int k=1;k<3;++k)if(node.box.hi[k]-node.box.lo[k]>node.box.hi[axis]-node.box.lo[axis])axis=k;
  int middle=(begin+end)/2;
  std::nth_element(order.begin()+begin,order.begin()+middle,order.begin()+end,[&](int a,int b){return triangles[a].lo[axis]+triangles[a].hi[axis]<triangles[b].lo[axis]+triangles[b].hi[axis];});
  int left=build(begin,middle),right=build(middle,end);nodes[index].left=left;nodes[index].right=right;return index;
 }
 static double bound(const Bounds&box,const double*a,const double*b){
  // Exact minimum of distance(segment(t), AABB)^2 on its piecewise quadratic intervals.
  double cuts[8]={0.,1.};int count=2;
  for(int k=0;k<3;++k){double d=b[k]-a[k];if(std::abs(d)<1e-20)continue;
   for(double edge:{box.lo[k],box.hi[k]}){double t=(edge-a[k])/d;if(t>0&&t<1)cuts[count++]=t;}}
  std::sort(cuts,cuts+count);double best=std::numeric_limits<double>::infinity();
  for(int i=0;i<count-1;++i){double low=cuts[i],high=cuts[i+1],mid=(low+high)*.5,A=0,B=0;
   for(int k=0;k<3;++k){double d=b[k]-a[k],x=a[k]+d*mid;if(x>=box.lo[k]&&x<=box.hi[k])continue;
    double offset=a[k]-(x<box.lo[k]?box.lo[k]:box.hi[k]);A+=d*d;B+=offset*d;}
   double t=A>0?std::max(low,std::min(high,-B/A)):low,value=0;
   for(int k=0;k<3;++k){double x=a[k]+(b[k]-a[k])*t;double gap=std::max({box.lo[k]-x,0.,x-box.hi[k]});value+=gap*gap;}
   best=std::min(best,value);
  }return best;
 }
public:
 SurfaceBvh(const double*t,int count):data(t,t+count*9),order(count),triangles(count){
  std::iota(order.begin(),order.end(),0);nodes.reserve(count*2);
  for(int i=0;i<count;++i)for(int k=0;k<3;++k){triangles[i].lo[k]=std::min({data[i*9+k],data[i*9+3+k],data[i*9+6+k]});triangles[i].hi[k]=std::max({data[i*9+k],data[i*9+3+k],data[i*9+6+k]});}
  build(0,count);
 }
 double query(const double*a,const double*b,double radius,double*out)const{
  double cutoff=radius+1e-10,best=cutoff*cutoff;int bestindex=std::numeric_limits<int>::max();bool found=false;
  std::vector<int> stack{0};double low[3],high[3];for(int k=0;k<3;++k){low[k]=std::min(a[k],b[k])-radius;high[k]=std::max(a[k],b[k])+radius;}
  while(!stack.empty()){
   int index=stack.back();stack.pop_back();const auto&node=nodes[index];
   if(bound(node.box,a,b)>best+1e-10*(1+best))continue;
   if(node.left>=0){double l=bound(nodes[node.left].box,a,b),r=bound(nodes[node.right].box,a,b);
    if(l<r){stack.push_back(node.right);stack.push_back(node.left);}else{stack.push_back(node.left);stack.push_back(node.right);}continue;}
   double packed[72],candidate[6],squared;int indices[8],count=0,selected;
   for(int i=node.begin;i<node.end;++i){int original=order[i];bool overlap=true;
    for(int k=0;k<3;++k)overlap=overlap&&triangles[original].hi[k]>=low[k]&&triangles[original].lo[k]<=high[k];
    if(overlap){indices[count]=original;std::memcpy(packed+count*9,data.data()+original*9,9*sizeof(double));++count;}}
   if(!count)continue;
   nearest_impl(packed,count,a,b,candidate,&squared,&selected);
   if(selected>=0&&(squared<best||(squared==best&&indices[selected]<bestindex))){best=squared;bestindex=indices[selected];found=true;std::memcpy(out,candidate,6*sizeof(double));}
  }return found?std::sqrt(best):std::numeric_limits<double>::infinity();
 }
};
extern "C" void* build_surface_bvh(const double*t,int count){try{return count>0?new SurfaceBvh(t,count):nullptr;}catch(...){return nullptr;}}
extern "C" void destroy_surface_bvh(void*p){delete static_cast<SurfaceBvh*>(p);}
extern "C" double query_surface_bvh(void*p,const double*a,const double*b,double radius,double*out){return static_cast<SurfaceBvh*>(p)->query(a,b,radius,out);}

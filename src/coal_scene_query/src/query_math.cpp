#include <cmath>
#include <algorithm>
#include <limits>
struct V {double x,y,z;V operator+(V b)const{return{x+b.x,y+b.y,z+b.z};}V operator-(V b)const{return{x-b.x,y-b.y,z-b.z};}V operator*(double a)const{return{x*a,y*a,z*a};}};
double dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
V cross(V a,V b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double clip(double a){return std::max(0.,std::min(1.,a));}
V read(const double*p){return{p[0],p[1],p[2]};}
extern "C" double nearest_surface(const double* triangles,int count,const double* av,const double* bv,double* output){
 V a=read(av),b=read(bv),u=b-a;double aa=dot(u,u),best=std::numeric_limits<double>::infinity();V bestpoint{},bestnormal{};
 auto offer=[&](V q,V p,V n){double d=dot(q-p,q-p);if(d<best){best=d;bestpoint=q;bestnormal=n;}};
 for(int i=0;i<count;++i){const double* data=triangles+i*9;V v[3]={read(data),read(data+3),read(data+6)};V e1=v[1]-v[0],e2=v[2]-v[0],n=cross(e1,e2);double nn=dot(n,n);
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
 double norm=std::sqrt(dot(bestnormal,bestnormal));output[0]=bestpoint.x;output[1]=bestpoint.y;output[2]=bestpoint.z;output[3]=norm?bestnormal.x/norm:0;output[4]=norm?bestnormal.y/norm:0;output[5]=norm?bestnormal.z/norm:0;return std::sqrt(best);
}

#include "radar/infer/detector.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
namespace radar {
cv::Mat Letterbox(const cv::Mat& bgr, int new_shape, LetterboxInfo& info, bool auto_mode, int stride) {
  const int h0=bgr.rows, w0=bgr.cols;
  const float r=std::min((float)new_shape/h0,(float)new_shape/w0);
  info.ratio=r; info.new_unpad_w=(int)std::round(w0*r); info.new_unpad_h=(int)std::round(h0*r);
  float dw=(float)(new_shape-info.new_unpad_w), dh=(float)(new_shape-info.new_unpad_h);
  if (auto_mode) { dw=std::fmod(dw,(float)stride); dh=std::fmod(dh,(float)stride); }
  dw*=0.5f; dh*=0.5f; info.pad_w=dw; info.pad_h=dh; info.out_w=new_shape; info.out_h=new_shape;
  cv::Mat resized=bgr;
  if (w0!=info.new_unpad_w||h0!=info.new_unpad_h) cv::resize(bgr,resized,cv::Size(info.new_unpad_w,info.new_unpad_h));
  int top=(int)std::round(dh-0.1f), bottom=(int)std::round(dh+0.1f), left=(int)std::round(dw-0.1f), right=(int)std::round(dw+0.1f);
  cv::Mat out; cv::copyMakeBorder(resized,out,top,bottom,left,right,cv::BORDER_CONSTANT,cv::Scalar(114,114,114));
  return out;
}
void ScaleBoxes(const LetterboxInfo& info, const cv::Size& orig, std::vector<cv::Rect2f>& boxes) {
  float gain=std::min((float)info.out_h/orig.height,(float)info.out_w/orig.width);
  float px=(info.out_w-orig.width*gain)/2.f, py=(info.out_h-orig.height*gain)/2.f;
  for (auto& b: boxes) {
    float x1=(b.x-px)/gain,y1=(b.y-py)/gain,x2=(b.x+b.width-px)/gain,y2=(b.y+b.height-py)/gain;
    x1=std::max(0.f,std::min(x1,(float)orig.width)); y1=std::max(0.f,std::min(y1,(float)orig.height));
    x2=std::max(0.f,std::min(x2,(float)orig.width)); y2=std::max(0.f,std::min(y2,(float)orig.height));
    b=cv::Rect2f(x1,y1,x2-x1,y2-y1);
  }
}
static float IoU(const cv::Rect2f& a,const cv::Rect2f& b){ float i=(a&b).area(); float u=a.area()+b.area()-i; return u>0?i/u:0; }
std::vector<Detection> NmsYoloV5(const float* pred,int num_pred,int num_attrs,float conf_thres,float iou_thres,int max_det,
  const std::vector<std::string>& names,const LetterboxInfo& lb,const cv::Size& orig) {
  const int nc=num_attrs-5; struct Cand{cv::Rect2f box; float conf; int cls;}; std::vector<Cand> cands;
  for (int i=0;i<num_pred;++i) {
    const float* r=pred+i*num_attrs; if (r[4]<conf_thres) continue;
    int bc=0; float bs=r[5]; for(int c=1;c<nc;++c) if(r[5+c]>bs){bs=r[5+c];bc=c;}
    float conf=r[4]*bs; if(conf<conf_thres) continue;
    cands.push_back({cv::Rect2f(r[0]-r[2]*0.5f,r[1]-r[3]*0.5f,r[2],r[3]),conf,bc});
  }
  std::sort(cands.begin(),cands.end(),[](auto&a,auto&b){return a.conf>b.conf;});
  if ((int)cands.size()>30000) cands.resize(30000);
  std::vector<Cand> kept; std::vector<char> rem(cands.size(),0);
  for (size_t i=0;i<cands.size();++i){ if(rem[i]) continue; kept.push_back(cands[i]); if((int)kept.size()>=max_det) break;
    for(size_t j=i+1;j<cands.size();++j){ if(rem[j]||cands[i].cls!=cands[j].cls) continue; if(IoU(cands[i].box,cands[j].box)>iou_thres) rem[j]=1; }}
  std::vector<cv::Rect2f> boxes; for(auto&k:kept) boxes.push_back(k.box); ScaleBoxes(lb,orig,boxes);
  std::vector<Detection> out;
  for (size_t i=0;i<kept.size();++i){
    Detection d; d.cls_id=kept[i].cls; d.conf=kept[i].conf;
    d.cls=(d.cls_id>=0&&d.cls_id<(int)names.size())?names[d.cls_id]:std::to_string(d.cls_id);
    auto&b=boxes[i]; int x1=(int)std::round(b.x),y1=(int)std::round(b.y),x2=(int)std::round(b.x+b.width),y2=(int)std::round(b.y+b.height);
    d.xywh=cv::Rect(x1,y1,x2-x1,y2-y1); out.push_back(d);
  }
  return out;
}
}  // namespace radar

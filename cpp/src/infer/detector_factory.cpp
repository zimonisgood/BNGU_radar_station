#include "radar/infer/detector.hpp"
#include <cstring>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
namespace radar {
class YoloOnnxCv : public Detector {
 public:
  bool Init(const std::string& weights, const std::vector<std::string>& names, const DetectionConfig& cfg, std::string* err) override {
    names_=names; cfg_=cfg;
    try {
      net_=cv::dnn::readNetFromONNX(weights);
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const cv::Exception& e) { if (err) *err=std::string("DNN load: ")+e.what(); return false; }
    return true;
  }
  std::vector<Detection> Predict(const cv::Mat& bgr) override {
    LetterboxInfo lb; cv::Mat lbimg=Letterbox(bgr,cfg_.img_size,lb,false);
    cv::Mat rgb; cv::cvtColor(lbimg,rgb,cv::COLOR_BGR2RGB);
    cv::Mat blob; cv::dnn::blobFromImage(rgb,blob,1./255.,cv::Size(cfg_.img_size,cfg_.img_size),cv::Scalar(),false,false,CV_32F);
    net_.setInput(blob); cv::Mat out=net_.forward();
    int np=0,na=0; std::vector<float> buf; const float* data=nullptr;
    if (out.dims==3) {
      if (out.size[2]<=64) { np=out.size[1]; na=out.size[2]; buf.resize((size_t)np*na);
        for(int i=0;i<np;++i) std::memcpy(buf.data()+(size_t)i*na,out.ptr<float>(0,i),sizeof(float)*na); data=buf.data(); }
      else { na=out.size[1]; np=out.size[2]; buf.resize((size_t)np*na);
        for(int c=0;c<na;++c){ const float* s=out.ptr<float>(0,c); for(int i=0;i<np;++i) buf[(size_t)i*na+c]=s[i]; } data=buf.data(); }
    } else if (out.dims==2) { np=out.rows; na=out.cols; data=out.ptr<float>(0); } else return {};
    return NmsYoloV5(data,np,na,cfg_.conf_thres,cfg_.iou_thres,cfg_.max_det,names_,lb,bgr.size());
  }
  std::string BackendName() const override { return "opencv_dnn"; }
 private:
  cv::dnn::Net net_; std::vector<std::string> names_; DetectionConfig cfg_;
};
#ifdef RADAR_WITH_TENSORRT
#include "yolo_trt.hpp"
#endif
std::unique_ptr<Detector> CreateDetector(const std::string& pref_in, std::string* chosen, std::string* err) {
  std::string pref=pref_in.empty()?"auto":pref_in;
#ifdef RADAR_WITH_TENSORRT
  if (pref=="tensorrt"||pref=="auto") { if(chosen)*chosen="tensorrt"; return std::make_unique<YoloTrt>(); }
#else
  if (pref=="tensorrt") { if(err)*err="no TensorRT"; return nullptr; }
#endif
  if (pref=="opencv_dnn"||pref=="auto") { if(chosen)*chosen="opencv_dnn"; return std::make_unique<YoloOnnxCv>(); }
  if (err) *err="unknown backend"; return nullptr;
}
}  // namespace radar

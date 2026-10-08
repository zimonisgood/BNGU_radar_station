#include "radar/pipeline/roi_packer.hpp"
#include <algorithm>
namespace radar {
std::pair<std::vector<cv::Mat>, std::vector<std::vector<PackedRoi>>>
PackRoisToCanvas(const std::vector<cv::Mat>& rois, cv::Size canvas_size, int padding) {
  std::vector<cv::Mat> canvases; std::vector<std::vector<PackedRoi>> all;
  cv::Mat cur=cv::Mat::zeros(canvas_size.height, canvas_size.width, CV_8UC3);
  std::vector<PackedRoi> cur_rois; int x=0,y=0,row_h=0;
  for (int idx=0; idx<(int)rois.size(); ++idx) {
    const auto& roi=rois[idx]; if (roi.empty()) continue;
    int h=roi.rows,w=roi.cols,ph=h+2*padding,pw=w+2*padding;
    if (pw>canvas_size.width || ph>canvas_size.height) continue;
    if (x+pw>canvas_size.width) { x=0; y+=row_h; row_h=0; }
    if (y+ph>canvas_size.height) {
      canvases.push_back(cur.clone()); all.push_back(cur_rois);
      cur=cv::Mat::zeros(canvas_size.height, canvas_size.width, CV_8UC3);
      cur_rois.clear(); x=0; y=0; row_h=0;
    }
    cur(cv::Rect(x,y,pw,ph)).setTo(cv::Scalar(0,0,0));
    roi.copyTo(cur(cv::Rect(x+padding,y+padding,w,h)));
    PackedRoi pr; pr.roi_idx=idx; pr.rx=x+padding; pr.ry=y+padding; pr.rw=w; pr.rh=h; pr.ox=x; pr.oy=y;
    cur_rois.push_back(pr); x+=pw; row_h=std::max(row_h,ph);
  }
  if (!cur_rois.empty()) { canvases.push_back(cur); all.push_back(cur_rois); }
  return {canvases, all};
}
}  // namespace radar

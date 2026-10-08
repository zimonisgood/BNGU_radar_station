#pragma once
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
namespace radar {
class ScopedTimer {
 public:
  explicit ScopedTimer(double* accum) : accum_(accum), t0_(Clock::now()) {}
  ~ScopedTimer() { if (accum_) *accum_ += Ms(); }
  double Ms() const { return std::chrono::duration<double,std::milli>(Clock::now()-t0_).count(); }
 private:
  using Clock=std::chrono::steady_clock; double* accum_; Clock::time_point t0_;
};
class PerfStats {
 public:
  void Reset(){ std::lock_guard<std::mutex> lk(mu_); tot_.clear(); cnt_.clear(); frames_=0; }
  void Add(const std::string& k,double ms){ std::lock_guard<std::mutex> lk(mu_); tot_[k]+=ms; cnt_[k]+=1; }
  void Tick(){ std::lock_guard<std::mutex> lk(mu_); ++frames_; }
  double Mean(const std::string& k) const { std::lock_guard<std::mutex> lk(mu_); auto it=tot_.find(k); if(it==tot_.end())return 0; int c=cnt_.at(k); return c?it->second/c:0; }
  int Frames() const { std::lock_guard<std::mutex> lk(mu_); return frames_; }
  std::string Summary(double wall_s) const {
    std::lock_guard<std::mutex> lk(mu_);
    double fps=wall_s>0?frames_/wall_s:0;
    std::string o="frames="+std::to_string(frames_)+" wall_s="+std::to_string(wall_s)+" e2e_fps="+std::to_string(fps)+"\n";
    for(auto& kv:tot_){ int c=cnt_.at(kv.first); o+="  "+kv.first+"_ms="+std::to_string(c?kv.second/c:0)+"\n"; }
    return o;
  }
 private:
  mutable std::mutex mu_; std::unordered_map<std::string,double> tot_; std::unordered_map<std::string,int> cnt_; int frames_=0;
};
} // namespace radar

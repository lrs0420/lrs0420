- 👋 Hi, I’m @lrs0420
- 👀 I’m interested in ...
- 🌱 I’m currently learning ...
- 💞️ I’m looking to collaborate on ...
- 📫 How to reach me ...

<!---
lrs0420/lrs0420 is a ✨ special ✨ repository because its `README.md` (this file) appears on your GitHub profile.
You can click the Preview link to take a look at your changes.
--->
//主要是PathData类的实现
/**
 * @file path_data.cc
 **/
 
#include "modules/planning/common/path/path_data.h"
 
#include <algorithm>
 
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "cyber/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/planning_gflags.h"
 
namespace apollo {
namespace planning {
 
using apollo::common::PathPoint;
using apollo::common::PointENU;
using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::PointFactory;
 
//设置离散路径，将参数赋给数据成员discretized_path_，同时调用XYToSL
//将XYToSL函数的结果放到PathData类数据成员frenet_path_里
bool PathData::SetDiscretizedPath(DiscretizedPath path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  discretized_path_ = std::move(path);
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}
 
//设置Frenet系路径 将参数frenet_path赋值给数据成员frenet_path_
//同时调用SLToXY，并将转化的XY坐标系路径放入类数据成员discretized_path_
bool PathData::SetFrenetPath(FrenetFramePath frenet_path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  frenet_path_ = std::move(frenet_path);
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AERROR << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}
 
//设置路径点决策参考，将输入参数path_point_decision_guide传给
//数据成员path_point_decision_guide_
bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathPointType, double>>
        path_point_decision_guide) {
  //若发现数据成员道路参考线为空，报错
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set path_point_decision_guide when reference line is "
              "nullptr. ";
    return false;
  }
  //发现数据成员frenet_path_和discretized_path_只要有一个为空就报错
  //检查frenet路径或xy路径是否为空
  if (frenet_path_.empty() || discretized_path_.empty()) {
    AERROR << "Should NOT set path_point_decision_guide when frenet_path or "
              "world frame trajectory is empty. ";
    return false;
  }
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}
 
//返回PathData类的数据成员——离散路径discretized_path_
const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}
 
//返回PathData类的数据成员——frenet系路径frenet_path_
const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}
 
//返回PathData类的数据成员——路径点决策参考path_point_decision_guide_
const std::vector<std::tuple<double, PathData::PathPointType, double>>
    &PathData::path_point_decision_guide() const {
  return path_point_decision_guide_;
}
 
//检查PathData路径类数据是否为空
//就是检查xy路径对象vector discretized_path_和SL路径对象frenet_path_ vector
//是否都不为空
bool PathData::Empty() const {
  return discretized_path_.empty() && frenet_path_.empty();
}
 
//设置参考线，先清空之前的PathData类对象的数据，然后将输入参数reference_line赋值给
//数据成员reference_line_道路参考线
void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}
 
//去类数据成员discretized_path_离散路径上寻找给定纵向位置s对应的插值点
common::PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}
 
//获取路径点用参考纵向位置ref_s? 函数结果放入指针path_point中
//其实这个函数的作用就是给定纵向位置ref_s去frenet系上找到最近点，然后根据最近点索引
//去离散路径上discretized_path_求出对应的离散路径点，貌似功能有点重复？
bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
  ACHECK(reference_line_);
  //判断是否相等，离散路径discretized_path_点的个数和frenet系路径点的个数是否相等
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  //如果给定的纵向位置参数ref_s是小于0的则报错
  if (ref_s < 0) {
    AERROR << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  //如果给定纵向位置ref_s是大于frenet系路径最后一个点的纵向位置s的，也报错
  if (ref_s > frenet_path_.back().s()) {
    AERROR << "ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.back().s() << "].";
    return false;
  }
  //索引什么索引？
  uint32_t index = 0;
  //定义一个小正数，是精度误差容许项？
  const double kDistanceEpsilon = 1e-3;
  //遍历frenet系路径点
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i) {
    //实际上frenet_path_和discretized_path_都是继承路径点或frenet系路径点vector类数据结构
    //对于vector序列容器，.at(0)表示获取vector的第一个元素，相比[]会自动检查索引是否越界
    //如果参考点s距离frenet系路径的第i个点的s间的距离小于小正数kDistanceEpsilon
    //那么路径点就从离散路径的第i个进行拷贝并直接返回
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }
    //如果frenet系路径的第i个点的纵向位置s < 给定参数参考纵向位置ref_s
    //就是ref_s大于第i个frenet点s但是又小于第i+1个frenet点的s，介于第i个和第i+1个点之间
    if (frenet_path_.at(i).s() < ref_s && ref_s <= frenet_path_.at(i + 1).s()) {
      //当ref_s介于第i个点和第i+1个点之间，那么取index为第i个点
      index = i;
      //跳出循环
      break;
    }
  }
  //由上面可知index就是当ref_s位于frenet系路径第i,i+1个点的路径之间，index取i
  //即index为ref_s在frenet系路径中的纵向最近点索引
  //定义r为给定(纵向位置ref_s减去frenet系上最近点s)/(第i+1,i个点之间的纵向距离)
  //其实r就是ref_s在i,i+1之间的距离比例，如r=0.5，则ref_s对应第i,i+1点的中点
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());
 
  //离散路径的纵向位置discretized_path_s 等于离散路径discretized_path_上
  //最近点s + r * 第i,i+1点纵向距离
  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());
  //用离散路径点s去插值出离散路径点，再拷贝给指针path_point，传出
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));
 
  return true;
}
 
//路径点的清空函数
//清空类数据成员vector discretized_path_,frenet_path_,path_point_decision_guide_
//清空类数据成员path_reference_,reference_line_ 
void PathData::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  path_point_decision_guide_.clear();
  path_reference_.clear();
  reference_line_ = nullptr;
}
 
//路径点类的数据成员DebugString
std::string PathData::DebugString() const {
  const auto limit =
      std::min(discretized_path_.size(), //离散路径点个数
                //FLAGS_trajectory_point_num_for_debug gflags用法
                //去除gflag文件中trajectory_point_num_for_debug的值，为10
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
 
  return absl::StrCat(
      "[\n",
      //记录第一个点和至少第10个点之后的点信息转换为字符串输出用于debug
      absl::StrJoin(discretized_path_.begin(),
                    discretized_path_.begin() + limit, ",\n",
                    apollo::common::util::DebugStringFormatter()),
      "]\n");
}
 
///*
   * 转化frenet路径到笛卡尔坐标系路径，通过道路参考线(SL坐标转化为xy坐标)
   */
  //转换结果存放到参数discretized_path，调用时通过引用变量将函数结果传出
bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  std::vector<common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    const common::SLPoint sl_point =
        PointFactory::ToSLPoint(frenet_point.s(), frenet_point.l());
    common::math::Vec2d cartesian_point;
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    const ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());
    const double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    ADEBUG << "frenet_point: " << frenet_point.ShortDebugString();
    const double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());
 
    double s = 0.0;
    double dkappa = 0.0;
    if (!path_points.empty()) {
      common::math::Vec2d last = PointFactory::ToVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      s = path_points.back().s() + distance;
      dkappa = (kappa - path_points.back().kappa()) / distance;
    }
    path_points.push_back(PointFactory::ToPathPoint(cartesian_point.x(),
                                                    cartesian_point.y(), 0.0, s,
                                                    theta, kappa, dkappa));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));
 
  return true;
}
 
//将笛卡尔坐标系路径转化到frenet路径存放到frenet_path，调用时通过引用变量将函数结果传出
bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  ACHECK(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path) {
    common::FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL(path_point, &sl_point)) {
        AERROR << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}
 
//裁剪掉frenet路径上frenet_point点左边的路径,trim 修剪得意思
bool PathData::LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point) {
  ACHECK(reference_line_);
  //frenet系坐标点FrenetFramePoint类的vector对象frenet_frame_points
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  //frenet_frame_points首先把给定的frenet系路径点frenet_point塞到末尾
  frenet_frame_points.emplace_back(frenet_point);
 
  //遍历frenet_path_离散路径上的每一个frenet点,如果遍历点的s在给定点附近就跳过
  //感觉这个if有点多余
  for (const common::FrenetFramePoint fp : frenet_path_) {
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    //如果遍历点的s大于给定点的s的点则塞到frenet_frame_points里
    //这样遍历完后就在只剩下比给定点s大的点
    if (fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(fp);
    }
  }
  //最后遍历完后，将frenet_frame_points直接move到FrenetFramePath对象里，然后
  //将这个被move的FrenetFramePath对象作为参数调用SetFrenetPath，将裁剪后的frenet轨迹
  //放入PathData类数据成员对象frenet_path_ frenet系路径里
  SetFrenetPath(FrenetFramePath(std::move(frenet_frame_points)));
  return true;
}
 
//更新frenet系路径，用给定的参考线reference_line赋值给PathData类成员参考线reference_line_ 
//然后再用调用SetDiscretizedPath，里面会调用XYToSL()函数，从而更新frenet路径frenet_path_ 
bool PathData::UpdateFrenetFramePath(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}
 
//用给定的字符传设定PathData类数据成员———路径标签path_label_ 
void PathData::set_path_label(const std::string &label) { path_label_ = label; }
 
//获取Pathdata 路径数据类对象的路径标签对象path_label_
const std::string &PathData::path_label() const { return path_label_; }
 
//获取参考路径：PathPoint类数组vector对象path_reference_
const std::vector<PathPoint> &PathData::path_reference() const {
  return path_reference_;
}
 
//用给定的参考路径去赋值PathData类成员里的参考路径，直接move，而非拷贝
void PathData::set_path_reference(
    const std::vector<PathPoint> &path_reference) {
  path_reference_ = std::move(path_reference);
}
 
}  // namespace planning
}  // namespace apollo
————————————————
版权声明：本文为CSDN博主「wujiangzhu_xjtu」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/weixin_39199083/article/details/123538974

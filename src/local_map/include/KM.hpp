#ifndef KMALGORITHM_H
#define KMALGORITHM_H

#include <third_party/Hungarian.hpp>
#include <set>
#include <limits>

#include "ellipse.hpp"

//这块原来的代码太乱了。。。不保证没有改错逻辑
class KMAlgorithm
{
public:
  KMAlgorithm();
  
  void tracking(std::vector<Ellipse>& input_vector);
  
  // 设置算法参数
  void set_parameters(float match_thresh = 1.0f, float hist_thresh = 0.2f, int max_missing_frames = 5);
  
  // 清空历史记录
  void clear_history();

private:
  void check_in_his_list(Ellipse& obs);
  int get_new_unique_id();
  void cleanup_old_targets();
  
  std::vector<Ellipse> last_label_list;
  std::vector<Ellipse> old_label_list;
  
  // ID管理
  std::set<int> used_ids;
  int next_available_id;
  
  // 可配置参数
  float match_distance_threshold;
  float history_match_threshold;
  int max_missing_frames;
  
  // 帧计数器用于清理旧目标
  int frame_count;
};

KMAlgorithm::KMAlgorithm() 
  : next_available_id(1), 
    match_distance_threshold(1.0f),
    history_match_threshold(0.2f),
    max_missing_frames(5),
    frame_count(0)
{
}

void KMAlgorithm::set_parameters(float match_thresh, float hist_thresh, int max_missing)
{
  match_distance_threshold = match_thresh;
  history_match_threshold = hist_thresh;
  max_missing_frames = max_missing;
}

void KMAlgorithm::clear_history()
{
  last_label_list.clear();
  old_label_list.clear();
  used_ids.clear();
  next_available_id = 1;
  frame_count = 0;
}

int KMAlgorithm::get_new_unique_id()
{
  while (used_ids.count(next_available_id)) {
    next_available_id++;
  }
  used_ids.insert(next_available_id);
  return next_available_id++;
}

void KMAlgorithm::cleanup_old_targets()
{
  // 定期清理长时间未更新的目标（可选功能，暂时简化）
  frame_count++;
}

void KMAlgorithm::tracking(std::vector<Ellipse>& input_vector)
{
  int new_size = input_vector.size();
  int last_size = last_label_list.size();

  if (new_size == 0) {
    last_label_list.clear();
    return;
  }

  if (last_size == 0) {
    // 第一帧或重启情况，检查历史记录
    for (auto& input : input_vector) {
      check_in_his_list(input);
    }
  }
  else {
    // 正常匹配情况：使用匈牙利算法
    std::vector<std::vector<double>> dis_matrix(new_size, std::vector<double>(last_size));
    
    // 计算距离矩阵并进行数据验证
    bool has_valid_data = false;
    for (int i = 0; i < new_size; i++) {
      for (int j = 0; j < last_size; j++) {
        double dist = calculate_dis(input_vector[i], last_label_list[j]);
        
        // 数据验证：确保距离值有效
        if (dist != dist || dist < 0 || dist == std::numeric_limits<double>::infinity()) {
          // 无效距离，使用默认大值
          dis_matrix[i][j] = 1000.0;
        } else {
          dis_matrix[i][j] = dist;
          has_valid_data = true;
        }
      }
    }

    // 如果所有距离都无效，降级到历史匹配
    if (!has_valid_data) {
      std::cerr << "Warning: All distances invalid, falling back to history matching" << std::endl;
      for (auto& input : input_vector) {
        check_in_his_list(input);
      }
    }
    else {
      // 使用匈牙利算法求解最优分配
      HungarianAlgorithm hun_alg;
      std::vector<int> assignment;
      double cost = hun_alg.Solve(dis_matrix, assignment);
    
          // 应用分配结果
      for (int i = 0; i < new_size; i++) {
        if (assignment[i] != -1 && 
            assignment[i] < last_size && 
            dis_matrix[i][assignment[i]] < match_distance_threshold) {
          // 成功匹配到上一帧的目标
          input_vector[i].label = last_label_list[assignment[i]].label;
        }
        else {
          // 未匹配成功，检查历史记录或分配新ID
          check_in_his_list(input_vector[i]);
        }
      }
    }
  }
  
  // 更新上一帧列表
  last_label_list = input_vector;
  cleanup_old_targets();
}

void KMAlgorithm::check_in_his_list(Ellipse& input)
{
  int his_size = old_label_list.size();
  
  if (his_size == 0) {
    // 历史记录为空，分配新ID
    input.label = get_new_unique_id();
    old_label_list.emplace_back(input);
    return;
  }

  int best_match_index = -1;
  float min_dis = std::numeric_limits<float>::max();
  
  // 在历史记录中寻找最佳匹配
  for (int j = 0; j < his_size; j++) {
    float dis = calculate_dis(input, old_label_list[j]);
    if (dis < min_dis) {
      min_dis = dis;
      best_match_index = j;
    }
  }

  if (best_match_index != -1 && min_dis < history_match_threshold) {
    // 找到历史匹配目标
    input.label = old_label_list[best_match_index].label;
    old_label_list[best_match_index] = input;  // 更新历史记录
  }
  else {
    // 没有找到匹配，分配新ID
    input.label = get_new_unique_id();
    old_label_list.emplace_back(input);
  }
}

#endif
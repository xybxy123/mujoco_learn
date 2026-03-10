#include "leg_ik.h"

void leg_ik(float x, float y, float *angle_out) {

  // 1. 预计算常数和平方项，避免重复乘法
  const float l1_sq = L1 * L1;
  const float l2_sq = L2 * L2;

  // 用 r_sq 替代重复的 x*x + y*y
  float r_sq = x * x + y * y;

  // 2. 奇异位保护（限制目标点在最大/最小腿长圆环工作空间内，防止 acosf 产生
  // NaN）
  float max_r_sq = (L1 + L2) * (L1 + L2) - 1e-6f;
  float min_r_sq = (L1 - L2) * (L1 - L2) + 1e-6f;
  if (r_sq > max_r_sq)
    r_sq = max_r_sq;
  else if (r_sq < min_r_sq)
    r_sq = min_r_sq;

  float r = sqrtf(r_sq);

  // 3. 计算中间变量，加入 (-1.0f, 1.0f)
  // 边界截断，彻底消除因浮点数误差引发的几何折叠炸飞
  float cos_shank = (l1_sq + l2_sq - r_sq) / (2.0f * L1 * L2);
  if (cos_shank > 1.0f)
    cos_shank = 1.0f;
  else if (cos_shank < -1.0f)
    cos_shank = -1.0f;
  float shank = 180.0f - acosf(cos_shank);

  float cos_fai = (l1_sq + r_sq - l2_sq) / (2.0f * L1 * r);
  if (cos_fai > 1.0f)
    cos_fai = 1.0f;
  else if (cos_fai < -1.0f)
    cos_fai = -1.0f;
  float fai = acosf(cos_fai);

  // 4. 使用 atan2f 自动处理全象限判断及 x=0 时的除零问题，直接替代原先庞大的
  // if-else
  float beta;
  if (x == 0) {
    beta = 90;
  } else {
    beta = atan2f(y, x);
  }
  float ham = beta + fai;

  // 5. 转换为角度并赋值
  angle_out[0] = ham * 180.0f / (float)PI;
  angle_out[1] = shank * 180.0f / (float)PI;

  angle_out[0] = -angle_out[0] + 90;
  angle_out[1] = -angle_out[1];

  return;
}
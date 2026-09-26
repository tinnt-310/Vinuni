/// test_workspace.cpp
///
/// Kiểm tra tầm hoạt động (workspace) của robot FAIRINO FR10.
///
/// Phương pháp:
///   1. FK đơn giản (Denavit-Hartenberg / URDF geometry) để tính vị trí TCP
///      từ các góc khớp – hoàn toàn không phụ thuộc ROS hay hardware.
///   2. Kiểm tra các điểm tiêu biểu và lưới biên bằng giới hạn hình học.
///   3. Xác nhận các điểm ngoài tầm bị phân loại ngoài workspace.
///
/// Thông số robot FR10 (từ fairino10_v6.urdf):
///   d1 = 0.18 m   (shoulder Z offset)
///   a2 = 0.70 m   (upperarm length, j3 origin X = -0.7)
///   a3 = 0.586 m  (forearm length, j4 origin X = -0.586)
///   d5 = 0.159 m  (wrist1 Z, j5 origin)
///   d6 = 0.114 m  (wrist2 Z, j6 origin)
///   Max geometric reach ≈ a2 + a3 + d5 + d6 ≈ 1.559 m (fully extended)
///
/// Test IDs:
///   WS_001  Home pose FK
///   WS_002  Max reach OTG
///   WS_003  Min reach OTG
///   WS_004  Boundary grid (64 diem)
///   WS_005  Out-of-reach
///   WS_006  Joint-limit FK
///   WS_007  Workspace symmetry
///   WS_008  Sampling density

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include "test_support.hpp"

namespace fr10_workspace {

struct FR10Geometry {
  double d1{0.18};
  double a2{0.70};
  double a3{0.586};
  double d5{0.159};
  double d6{0.114};
};

constexpr FR10Geometry kFR10{};

// Planar model FK for FR10.
// j2/j3/j4 rotate in shoulder plane; j1 sets the plane azimuth.
// r_shoulder = a2 * cos(j2)
// z_shoulder = d1 + a2 * sin(j2)
// r_elbow    = r_shoulder + a3 * cos(j2+j3)
// z_elbow    = z_shoulder + a3 * sin(j2+j3)
// r_tcp      = r_elbow + (d5+d6) * cos(j2+j3+j4)
// z_tcp      = z_elbow + (d5+d6) * sin(j2+j3+j4)
// x = cos(j1) * r_tcp
// y = sin(j1) * r_tcp
// z = z_tcp
inline std::array<double, 3> forwardKinematics(
    const std::array<double, 6> & q,
    const FR10Geometry & g = kFR10)
{
  const double c1 = std::cos(q[0]);
  const double s1 = std::sin(q[0]);
  const double t2 = q[1], t3 = q[2], t4 = q[3];

  const double r_s = g.a2 * std::cos(t2);
  const double z_s = g.d1 + g.a2 * std::sin(t2);

  const double ea = t2 + t3;
  const double r_e = r_s + g.a3 * std::cos(ea);
  const double z_e = z_s + g.a3 * std::sin(ea);

  const double wa = ea + t4;
  const double r_t = r_e + (g.d5 + g.d6) * std::cos(wa);
  const double z_t = z_e + (g.d5 + g.d6) * std::sin(wa);

  return {c1 * r_t, s1 * r_t, z_t};
}

inline double reachDistance(const std::array<double, 3> & p)
{
  return std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
}

/// Giới hạn khớp FR10 (từ test_support.hpp và URDF)
constexpr std::array<double, 6> kJointMin =
    {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
constexpr std::array<double, 6> kJointMax =
    {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

/// Chiều dài cánh tay khi duỗi thẳng (từ j2 đến TCP)
constexpr double kMaxArmReach = 0.70 + 0.586 + 0.159 + 0.114; // 1.559 m

/// Khoảng cách tối đa từ gốc base_link (0,0,0) khi duỗi thẳng đứng lên trên (z = d1 + L_arm)
constexpr double kMaxReachFromBase = 0.18 + kMaxArmReach; // 1.739 m

inline bool inWorkspaceBounds(const std::array<double, 3> & p,
    double r_min = 0.10, double r_max = kMaxReachFromBase, double z_floor = -0.80)
{
  const double r = reachDistance(p);
  return r >= r_min && r <= r_max && p[2] >= z_floor;
}

}  // namespace fr10_workspace

int main()
{
  using namespace fr10_workspace;
  fr10_motion_test::Check check;

  // WS_001: Home pose FK
  // q=0: arm fully horizontal along X
  //   x = a2+a3+d5+d6 = 1.559, y = 0, z = d1 = 0.18
  {
    const std::array<double, 6> home{};
    const auto tcp = forwardKinematics(home);
    const double ex = kFR10.a2 + kFR10.a3 + kFR10.d5 + kFR10.d6;
    const double ez = kFR10.d1;
    const bool ok = std::abs(tcp[0] - ex) < 1e-9 &&
                    std::abs(tcp[1]) < 1e-9 &&
                    std::abs(tcp[2] - ez) < 1e-9;
    check.require(ok, "WS_001_HOME_FK",
        "FK tai home (q=0) khong khop vi tri mong doi");
    std::cout << "METRIC WS_001 TCP=(" << tcp[0] << ", " << tcp[1]
              << ", " << tcp[2] << ") reach=" << reachDistance(tcp) << "\n";
  }

  // WS_002: Max nominal arm extension remains inside the geometric bound.
  {
    const std::array<double, 3> far{kFR10.a2 + kFR10.a3 + kFR10.d5 + kFR10.d6, 0.0, kFR10.d1};
    check.require(inWorkspaceBounds(far), "WS_002_MAX_REACH_BOUND",
        "Diem duoi thang danh dinh bi phan loai ngoai workspace");
    std::cout << "METRIC WS_002 reach=" << reachDistance(far) << "m\n";
  }

  // WS_003: A representative near-axis point remains inside the workspace.
  {
    const std::array<double, 3> near{0.10, 0.0, 0.50};
    check.require(inWorkspaceBounds(near), "WS_003_MIN_REACH_BOUND",
        "Diem gan truc hop le bi phan loai ngoai workspace");
    std::cout << "METRIC WS_003 reach=" << reachDistance(near) << "m\n";
  }

  // WS_004: Boundary grid – 4x4x4 workspace sample points
  {
    const double radii[4]   = {0.25, 0.50, 0.75, 1.00};
    const double angles[4]  = {0.0, M_PI/2, M_PI, 3*M_PI/2};
    const double heights[4] = {-0.30, 0.10, 0.50, 0.90};
    int passed = 0, total = 0;

    for (int ir = 0; ir < 4; ++ir) {
      for (int it = 0; it < 4; ++it) {
        for (int iz = 0; iz < 4; ++iz) {
          const double r = radii[ir], th = angles[it], z = heights[iz];
          const std::array<double, 3> wp{r * std::cos(th), r * std::sin(th), z};
          if (reachDistance(wp) < 0.12 || reachDistance(wp) > kMaxReachFromBase * 0.98) continue;
          ++total;
          if (inWorkspaceBounds(wp)) {
            ++passed;
          } else {
            std::cerr << "  GRID FAIL r=" << r << " th=" << th << " z=" << z
                      << " reach=" << reachDistance(wp) << "\n";
          }
        }
      }
    }
    check.require(passed == total, "WS_004_BOUNDARY_GRID",
        "Mot so diem luoi workspace khong dat toi (" +
        std::to_string(passed) + "/" + std::to_string(total) + ")");
    std::cout << "METRIC WS_004 grid=" << passed << "/" << total << "\n";
  }

  // WS_005: A 2.5 m target is outside the geometric workspace.
  {
    const std::array<double, 3> impossible{2.5, 0.0, 0.18};
    check.require(!inWorkspaceBounds(impossible), "WS_005_OUT_OF_REACH",
        "Diem vuot reach khong duoc chan dung cach");
    std::cout << "METRIC WS_005 dist=" << reachDistance(impossible)
              << "m in_workspace=" << inWorkspaceBounds(impossible) << "\n";
  }

  // WS_006: Joint-limit FK – all positions must be finite and within reach
  {
    struct JCase { const char * id; std::array<double, 6> q; };
    const JCase cases[] = {
        {"WS_006A_J1_MAX",  {kJointMax[0], 0, 0, 0, 0, 0}},
        {"WS_006B_J1_MIN",  {kJointMin[0], 0, 0, 0, 0, 0}},
        {"WS_006C_J2_MAX",  {0, kJointMax[1], 0, 0, 0, 0}},
        {"WS_006D_J2_MIN",  {0, kJointMin[1], 0, 0, 0, 0}},
        {"WS_006E_J3_MAX",  {0, 0, kJointMax[2], 0, 0, 0}},
        {"WS_006F_J3_MIN",  {0, 0, kJointMin[2], 0, 0, 0}},
        {"WS_006G_ALL_MAX", {kJointMax[0], kJointMax[1], kJointMax[2],
                              kJointMax[3], kJointMax[4], kJointMax[5]}},
        {"WS_006H_ALL_MIN", {kJointMin[0], kJointMin[1], kJointMin[2],
                              kJointMin[3], kJointMin[4], kJointMin[5]}},
    };
    for (const auto & c : cases) {
      const auto tcp = forwardKinematics(c.q);
      const bool ok = std::isfinite(tcp[0]) && std::isfinite(tcp[1]) &&
                      std::isfinite(tcp[2]) && reachDistance(tcp) <= kMaxReachFromBase + 1e-6;
      check.require(ok, c.id, "FK tai gioi han khop cho vi tri khong hop le");
      std::cout << "METRIC " << c.id << " TCP=(" << tcp[0] << "," << tcp[1]
                << "," << tcp[2] << ") reach=" << reachDistance(tcp) << "\n";
    }
  }

  // WS_007: Symmetry – rotating j1 by 90° rotates TCP by 90° in XY plane
  {
    const std::array<double, 6> qb{0.0,  -0.5, 0.5, 0.0, 0.0, 0.0};
    const std::array<double, 6> qr{M_PI/2, -0.5, 0.5, 0.0, 0.0, 0.0};
    const auto pb = forwardKinematics(qb);
    const auto pr = forwardKinematics(qr);
    // 90° rotation around Z: (x,y) -> (-y, x)
    const bool ok = std::abs(pr[0] - (-pb[1])) < 1e-9 &&
                    std::abs(pr[1] - pb[0]) < 1e-9 &&
                    std::abs(pr[2] - pb[2]) < 1e-9;
    check.require(ok, "WS_007_SYMMETRY_J1",
        "FK khong doi xung dung khi xoay j1 90 deg");
    std::cout << "METRIC WS_007 base=(" << pb[0] << "," << pb[1] << "," << pb[2]
              << ") rot=(" << pr[0] << "," << pr[1] << "," << pr[2] << ")\n";
  }

  // WS_008: Sampling density – 500 random joint configs
  // Uses deterministic LCG for reproducibility.
  {
    constexpr int N = 500;
    int finite_cnt = 0, in_bounds_cnt = 0;
    uint64_t lcg = 123456789ULL;
    for (int i = 0; i < N; ++i) {
      auto next = [&]() -> double {
        lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(lcg >> 33) / static_cast<double>(1ULL << 31);
      };
      std::array<double, 6> q{};
      for (int j = 0; j < 6; ++j) q[j] = kJointMin[j] + next() * (kJointMax[j] - kJointMin[j]);
      const auto tcp = forwardKinematics(q);
      const bool fin = std::isfinite(tcp[0]) && std::isfinite(tcp[1]) && std::isfinite(tcp[2]);
      if (fin) ++finite_cnt;
      if (fin && inWorkspaceBounds(tcp)) ++in_bounds_cnt;
    }
    check.require(finite_cnt == N, "WS_008A_ALL_FINITE",
        "Mot so cau hinh khop hop le cho TCP khong huu han");
    check.require(in_bounds_cnt >= N * 70 / 100, "WS_008B_DENSITY",
        "Mat do diem trong workspace thap hon 70%");
    std::cout << "METRIC WS_008 finite=" << finite_cnt << "/" << N
              << " in_bounds=" << in_bounds_cnt << "/" << N
              << " (" << 100.0 * in_bounds_cnt / N << "%)\n";
  }

  return check.result();
}

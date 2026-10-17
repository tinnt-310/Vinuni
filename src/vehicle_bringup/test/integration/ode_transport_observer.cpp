// Read-only Gazebo transport observer in a separate process from gzserver.
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "ode_pose_frames.hpp"
#include <ignition/math/Quaternion.hh>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
std::atomic<bool> running{true};
void stop(int) { running.store(false); }
double wallSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
class Observer {
 public:
  Observer(const std::string &directory, std::string target)
      : out_(directory + "/transport_state.jsonl"), target_(std::move(target)) {
    if (!out_) throw std::runtime_error("Unable to open transport trace");
    out_ << std::setprecision(17);
    if (const char *reference = std::getenv("ODE_REFERENCE_LINK")) reference_=reference;
    if (std::getenv("ODE_OBSERVE_CONTACTS")) {
      contacts_.open(directory+"/transport_contacts.jsonl");
      if (!contacts_) throw std::runtime_error("Unable to open contacts trace");
      contacts_ << std::setprecision(17);
    }
  }
  void onContacts(const ::ConstContactsPtr &message) {
    if (!contacts_) return;
    contacts_ << "{\"wall\":" << wallSeconds() << ",\"sim\":"
              << message->time().sec()+message->time().nsec()*1e-9 << ",\"contacts\":[";
    for (int i=0;i<message->contact_size();++i) {
      const auto &contact=message->contact(i);
      if (i) contacts_ << ',';
      contacts_ << "{\"a\":\"" << contact.collision1() << "\",\"b\":\"" << contact.collision2()
                << "\",\"depths_m\":[";
      for (int j=0;j<contact.depth_size();++j) {if(j) contacts_ << ',';contacts_ << contact.depth(j);}
      contacts_ << "]}";
    }
    contacts_ << "]}\n";
  }
  void onPose(const ::ConstPosesStampedPtr &message) {
    ++messages_;
    const double wall = wallSeconds();
    if (wall - last_emit_ < 0.05) return;
    for (const auto &pose : message->pose()) {
      if (pose.name() != target_) continue;
      ignition::math::Pose3d world, reference_world;
      if (!resolveWorldPose(*message, target_, world)) continue;
      if (!reference_.empty() && !resolveWorldPose(*message, reference_, reference_world)) continue;
      const auto &p = world.Pos();
      const auto &q = world.Rot();
      const auto orientation = q;
      const auto rpy = orientation.Euler();
      double vx = 0, vy = 0, vz = 0, yaw_rate = 0;
      const double sim = message->time().sec() + 1e-9 * message->time().nsec();
      const double dt = sim - previous_sim_;
      if (previous_wall_ > 0 && dt > 0) {
        vx = (p.X() - previous_x_) / dt;
        vy = (p.Y() - previous_y_) / dt;
        vz = (p.Z() - previous_z_) / dt;
        yaw_rate = std::remainder(rpy.Z() - previous_yaw_, 2 * M_PI) / dt;
      }
      out_ << "{\"wall\":" << wall << ",\"sim\":" << sim
           << ",\"pose_frame\":\"world\",\"pose_schema\":2"
           << ",\"models\":[{\"name\":\"" << target_ << "\",\"position\":["
           << p.X() << ',' << p.Y() << ',' << p.Z() << "],\"rpy\":["
           << rpy.X() << ',' << rpy.Y() << ',' << rpy.Z() << "],\"linear\":["
           << vx << ',' << vy << ',' << vz << "],\"angular\":[0,0,"
           << yaw_rate << "],\"joints\":[]}],\"orientation_xyzw\":["
           << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W() << ']';
      if (!reference_.empty()) {
          const auto &p=reference_world.Pos();const auto &q=reference_world.Rot();
          out_ << ",\"reference_link\":{\"name\":\"" << reference_ << "\",\"position\":["
               << p.X() << ',' << p.Y() << ',' << p.Z() << "],\"orientation_xyzw\":["
               << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W() << "]}";
      }
      out_ << ",\"contacts\":[]}\n";
      // Make the first independent model sample visible to launch readiness
      // before any motion command is sent. Later samples remain buffered.
      if (target_samples_.load() == 0) out_.flush();
      previous_wall_ = last_emit_ = wall;
      ++target_samples_;
      previous_sim_ = sim;
      previous_x_ = p.X(); previous_y_ = p.Y(); previous_z_ = p.Z();
      previous_yaw_ = rpy.Z();
      break;
    }
  }
  uint64_t messages() const { return messages_.load(); }
  uint64_t targetSamples() const { return target_samples_.load(); }
 private:
  std::ofstream out_;
  std::ofstream contacts_;
  std::string reference_;
  std::string target_;
  std::atomic<uint64_t> messages_{0}, target_samples_{0};
  double last_emit_{0}, previous_wall_{0}, previous_sim_{0};
  double previous_x_{0}, previous_y_{0}, previous_z_{0}, previous_yaw_{0};
};
}  // namespace

int main(int argc, char **argv) {
  const char *directory = std::getenv("ODE_OUT");
  if (!directory) return 2;
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  gazebo::client::setup(argc, argv);
  const std::string world = std::getenv("ODE_WORLD_NAME")
      ? std::getenv("ODE_WORLD_NAME") : "default";
  const std::string target = std::getenv("ODE_TARGET_ENTITY")
      ? std::getenv("ODE_TARGET_ENTITY") : "my_mobile_manipulator";
  const std::string pose_topic = "/gazebo/" + world + "/pose/info";
  while (running.load()) {
    const auto topics = gazebo::transport::getAdvertisedTopics("");
    if (std::find(topics.begin(), topics.end(), pose_topic) != topics.end()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init(world);
  Observer observer(directory, target);
  auto subscription = node->Subscribe(pose_topic, &Observer::onPose, &observer);
  gazebo::transport::SubscriberPtr contact_subscription;
  if (std::getenv("ODE_OBSERVE_CONTACTS"))
    contact_subscription=node->Subscribe("/gazebo/"+world+"/physics/contacts", &Observer::onContacts,&observer);
  double last_retry = wallSeconds();
  while (running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const double now = wallSeconds();
    if (observer.messages() == 0 && now - last_retry > 3.0) {
      subscription.reset();
      node->Fini();
      node.reset(new gazebo::transport::Node());
      node->Init(world);
      subscription = node->Subscribe(pose_topic, &Observer::onPose, &observer);
      last_retry = now;
    }
  }
  std::fprintf(stderr, "pose transport messages=%lu target_samples=%lu\n",
               static_cast<unsigned long>(observer.messages()),
               static_cast<unsigned long>(observer.targetSamples()));
  subscription.reset();
  node->Fini();
  gazebo::client::shutdown();
  return 0;
}

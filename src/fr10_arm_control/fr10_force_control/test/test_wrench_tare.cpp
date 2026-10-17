#include <iostream>
#include <limits>
#include <stdexcept>
#include "fr10_force_control/wrench_tare.hpp"
#include "fr10_force_control/wrench_compensation.hpp"

int main() {
  using namespace fr10_force;
  bool ok = true;
  auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  WrenchTare tare;
  WrenchSample sample;
  sample.valid = true;
  sample.source_timestamp_s = 1.;
  auto observe = [&](const Wrench &residual, bool clear = true) {
    ++sample.sequence; sample.source_timestamp_s += .008;
    return tare.observe(sample, residual, clear);
  };
  const Wrench bias{{.12, -.08, .15}, {.002, 0., -.001}};
  for (int i = 0; i < 24; ++i)
    check(!observe(bias), "NO_EARLY_TARE");
  const auto result = observe(bias);
  check(result && std::abs(result->force_n[0] - .12) < 1e-12,
        "RESIDUAL_BIAS_QUALIFIED");
  const auto count = tare.sampleCount();
  check(!tare.observe(sample, bias, true) && tare.sampleCount() == count,
        "DUPLICATE_CANNOT_FINISH_TARE");
  check(!tare.observe(sample, bias, false) && tare.sampleCount() == 0,
        "MOTION_BETWEEN_UNIQUE_FT_SAMPLES_INVALIDATES_WINDOW");
  observe(bias, false);
  check(!tare.observe(sample, bias, true) && tare.sampleCount() == 0,
        "PACKET_OBSERVED_DURING_MOTION_CANNOT_LATER_ENTER_TARE");
  tare.reset();
  for (int i = 0; i < 30; ++i) check(!observe(bias, false), "CONTACT_OR_MOTION_REJECTED");
  check(tare.sampleCount() == 0, "NO_CONTACT_WINDOW_REQUIRED");
  for (int i = 0; i < 30; ++i)
    check(!observe({{0, 0, 6}, {}}), "CONTACT_FORCE_NOT_ZEROED");
  for (int i = 0; i < 30; ++i) {
    Wrench noisy = bias; noisy.force_n[0] += i % 2 ? 1. : -1.;
    check(!observe(noisy), "UNSTABLE_FT_NOT_CALIBRATED");
  }
  tare.reset();
  for (int i = 0; i < 20; ++i) observe(bias);
  sample.source_timestamp_s += .1;
  check(!observe(bias) && tare.sampleCount() == 0, "GAP_RESTARTS_WINDOW");
  observe(bias);
  sample.valid = false;
  check(!observe(bias) && tare.sampleCount() == 0, "INVALID_RESTARTS_WINDOW");
  sample.valid = true;
  tare.reset();
  // Payload remains physically attached: gravity is removed before tare.
  WrenchCompensationConfig cfg;
  cfg.gravity_enabled = cfg.gravity_calibrated = true;
  cfg.tool_mass_kg = 2.; cfg.tool_cog_m = {.1, 0., 0.};
  WrenchCompensator compensation(cfg);
  const Wrench loaded{{.12, -.08, -19.4633}, {.002, 1.96133, -.001}};
  std::optional<Wrench> qualified;
  for (int i = 0; i < 25; ++i) qualified = observe(compensation.apply(loaded));
  check(qualified && std::abs(qualified->force_n[2] - .15) < 1e-10,
        "TARE_WITH_ATTACHED_PAYLOAD");
  if (qualified) compensation.setBias(*qualified);
  check(norm(compensation.apply(loaded).force_n) < 1e-10,
        "PAYLOAD_REMOVED_ONCE_BIAS_REMOVED_ONCE");
  // A 125 Hz latest-only view of a symmetric 8 kHz load aliases its
  // final positive half-cycle into a false constant bias. Batch statistics
  // must recover the actual DC value and preserve the unfiltered variance.
  WrenchTare burst_tare;
  std::array<WrenchSample,64> burst;
  std::array<Wrench,64> residual;
  uint64_t sequence=0;
  double stamp=10.;
  auto burstObservation = [&](double amplitude, bool stationary=true) {
    for (std::size_t i=0;i<burst.size();++i) {
      burst[i].valid=true;burst[i].sequence=++sequence;
      burst[i].source_timestamp_s=(stamp+=.000125);
      residual[i]={{.03+(i%2?amplitude:-amplitude),0.,0.},{}};
    }
    return burst_tare.observeBatch(burst.data(),residual.data(),burst.size(),stationary);
  };
  for(unsigned tick=0;tick<24;++tick)
    check(!burstObservation(.2),"RAW_SUBSTEPS_CANNOT_REPLACE_25_CONTROL_OBSERVATIONS");
  const auto unbiased=burstObservation(.2);
  check(unbiased && std::abs(unbiased->force_n[0]-.03)<1e-12,
      "SYMMETRIC_BURST_TARE_RECOVERS_DC_WITHOUT_ALIASED_FINAL_SAMPLE");
  burst_tare.reset();
  for(unsigned tick=0;tick<40;++tick)
    check(!burstObservation(.5),"BATCH_MEAN_MUST_NOT_HIDE_EXCESSIVE_RAW_VARIANCE");
  burst_tare.reset();
  for(unsigned tick=0;tick<24;++tick) burstObservation(0.);
  burst.back().valid=false;burst.back().sequence=++sequence;
  burst.back().source_timestamp_s=(stamp+=.008);
  check(!burst_tare.observeBatch(burst.data(),residual.data(),burst.size(),true) &&
      burst_tare.sampleCount()==0,"LATE_INVALID_BATCH_SAMPLE_REVOKES_ENTIRE_WINDOW");
  burstObservation(0.);
  check(!burst_tare.observeBatch(burst.data(),residual.data(),burst.size(),false) &&
      burst_tare.sampleCount()==0,"ENCODER_ONLY_MOTION_REVOKES_BATCH_TARE");
  cfg.gravity_owner = GravityOwner::SOURCE;
  bool rejected = false;
  try { WrenchCompensator invalid(cfg); } catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, "DOUBLE_GRAVITY_REJECTED");
  cfg.gravity_enabled = false;
  WrenchCompensator upstream(cfg);
  check(std::abs(upstream.apply(bias).force_n[2] - .15) < 1e-12,
        "UPSTREAM_GRAVITY_NOT_SUBTRACTED_AGAIN");
  return ok ? 0 : 1;
}

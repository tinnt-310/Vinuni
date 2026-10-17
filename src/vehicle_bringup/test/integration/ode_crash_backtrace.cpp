// Diagnostic-only SIGSEGV trace for gzserver. Never load in production.
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

namespace {
void traceSegfault(int signal_number, siginfo_t *, void *) {
  constexpr char marker[] = "\n[ODE_CRASH_BACKTRACE] SIGSEGV\n";
  (void)::write(STDERR_FILENO, marker, sizeof(marker) - 1);
  void *frames[64];
  const int count = ::backtrace(frames, 64);
  ::backtrace_symbols_fd(frames, count, STDERR_FILENO);
  struct sigaction reset {};
  reset.sa_handler = SIG_DFL;
  ::sigemptyset(&reset.sa_mask);
  (void)::sigaction(signal_number, &reset, nullptr);
  (void)::raise(signal_number);
}

__attribute__((constructor)) void installTrace() {
  struct sigaction action {};
  action.sa_sigaction = traceSegfault;
  action.sa_flags = SA_SIGINFO | SA_RESETHAND;
  ::sigemptyset(&action.sa_mask);
  (void)::sigaction(SIGSEGV, &action, nullptr);
}
} // namespace

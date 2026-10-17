# Cooperative command lease, applied by the sole vehicle actuator controller.
# Receipt/expiry use monotonic time. Replayed sequence never renews the lease.
uint64 owner_id
uint64 sequence
bool release
builtin_interfaces/Duration lease_duration

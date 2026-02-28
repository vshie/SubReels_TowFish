# RadCam Spy Extension -- Instrumentation Recommendations

These recommendations are for the team working on the radcam spy BlueOS extension.
The spy already does a good job collecting camera SoC internals (CPU, memory, thermals,
ISP exposure, histogram errors, voltages). The additions below are prioritised by how
directly they help diagnose recording failures observed in the field.

**Important context**: The radcam spy extension is the single place for all system-level
monitoring. In addition to the camera SoC metrics it already collects, it should also
collect Pi4 host stats (see the new section below). The video recorder extension
deliberately does *not* duplicate this; it focuses on GStreamer pipeline health and
recording-specific diagnostics instead.

---

## High Priority -- RTSP Stream Health

### 1. RTSP active-client tracking

Monitor whether the RTSP server running on the camera has active client connections.
The camera's web API or procfs may expose connection counts, client IPs, and session
durations. Logging this lets us correlate "RTSP client dropped" events with file
corruption on the recorder side.

Suggested fields:

    rtsp_clients        (int)   number of connected RTSP clients
    rtsp_session_sec    (float) duration of the longest active session

### 2. H.265 encoder output frame counter

Track frames encoded per second from the ISP/encoder pipeline. The Rockchip `mpi`
(Media Process Interface) typically exposes encoded frame counts via sysfs or the
mpp debug interface. A sustained drop in encoded FPS is the most direct indicator
of the camera's pipeline falling behind.

Suggested fields:

    enc_fps             (float) frames encoded in the last 1-second window
    enc_frames_total    (int)   cumulative frames encoded since boot

### 3. Network interface TX statistics

Read `/proc/net/dev` or `/sys/class/net/eth0/statistics/` each sample and log:

    net_tx_bytes        (int)   cumulative TX bytes
    net_tx_packets      (int)   cumulative TX packets
    net_tx_errors       (int)   cumulative TX errors
    net_tx_dropped      (int)   cumulative TX dropped

A spike in `net_tx_errors` or `net_tx_dropped` directly indicates a network-layer
problem between the camera and the Pi4. Delta-encoding (reporting per-interval
changes) is also useful.

---

## Medium Priority -- Thermal and Stability Context

### 4. Explicit thermal throttling detection

The existing data already shows voltage drops (906 mV -> 882 mV at 82 C), which
strongly suggests thermal throttling. Make this explicit by reading:

- `/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq` -- current CPU frequency
- Rockchip-specific thermal zone files for throttle state

Suggested fields:

    cpu_freq_mhz        (int)   current CPU frequency
    throttled           (bool)  true if frequency is below max due to thermal policy

### 5. ISP histogram error context

`isp_histerror` values up to 47 were observed correlating with thermal peaks. To
understand whether these produce visible artifacts, also capture:

- ISP target brightness vs actual brightness (if the API exposes it)
- Whether auto-exposure is actively hunting (large `isp_exptime` swings)

This helps differentiate "benign histogram corrections" from "visible exposure
artifacts in the recorded video."

### 6. Uptime and reboot detection

Track system uptime (from `/proc/uptime`) to detect if the camera reboots
unexpectedly during a recording session. A sudden uptime reset while the recorder
is active is a clear indicator of a camera crash.

Suggested fields:

    uptime_sec          (float) seconds since last boot

---

## Lower Priority -- Deeper Diagnostics

### 7. IRQ and softirq pressure

Read `/proc/interrupts` and `/proc/softirqs` to detect if interrupt handling is
backing up. High softirq counts on NET_TX or NET_RX indicate the network stack is
under pressure. This is a second-order signal but useful for root-cause analysis.

### 8. DMA/CMA buffer pool status

If the Rockchip kernel exposes CMA or DMA-BUF allocation statistics (often via
`/proc/buddyinfo`, `/proc/meminfo` CMA fields, or debugfs), monitoring buffer pool
exhaustion can explain intermittent encoder stalls.

---

## High Priority -- Pi4 Host Monitoring

The radcam spy extension should also monitor the Pi4 that runs BlueOS and the
GStreamer recording pipeline. Currently there is zero telemetry about the Pi4's
health. These metrics should be collected at 1 Hz alongside the camera SoC data
(either interleaved in the same NDJSON stream with a `"source": "pi4"` field, or
in a parallel NDJSON file).

### 9. Pi4 CPU usage

Read `/proc/stat` and compute per-interval CPU percentage. Sustained > 90% usage
correlates with GStreamer buffer overflows and dropped frames.

Suggested fields:

    pi4_cpu_percent     (float) CPU usage over the last sample interval

### 10. Pi4 memory

Read `/proc/meminfo` for total and available memory. Low available memory can cause
GStreamer or other BlueOS extensions to be OOM-killed.

Suggested fields:

    pi4_mem_total_kb    (int)   total system memory
    pi4_mem_avail_kb    (int)   available memory

### 11. Pi4 CPU temperature

Read `/sys/class/thermal/thermal_zone0/temp`. The Pi4 throttles at 80 C; recording
quality degrades if the CPU is frequency-capped.

Suggested fields:

    pi4_cpu_temp_c      (float) CPU temperature in degrees C

### 12. Pi4 disk free space

Check free space on the recordings volume. When disk fills up, GStreamer writes
fail and the MP4 moov atom is never finalized.

Suggested fields:

    pi4_disk_free_mb    (float) free disk space in MB on the recordings partition

### 13. Pi4 network interface stats

Read `/proc/net/dev` for the interface connecting to the camera. RX errors or drops
on the Pi4 side complement the camera's TX stats (recommendation 3 above).

Suggested fields:

    pi4_net_rx_bytes    (int)   cumulative RX bytes
    pi4_net_rx_errors   (int)   cumulative RX errors
    pi4_net_rx_dropped  (int)   cumulative RX dropped

---

## Implementation Notes

- All new fields should be appended to the existing NDJSON schema (backward
  compatible; the recorder's analysis tools should ignore unknown fields).
- Fields that require delta computation (TX bytes, frame counts) should ideally
  be logged as both cumulative and per-interval values.
- Sampling rate should remain at 1 Hz to match the existing data.
- If any field is unavailable on the camera's platform, emit `null` rather than
  omitting the key, so downstream parsers can distinguish "not available" from
  "not yet implemented."

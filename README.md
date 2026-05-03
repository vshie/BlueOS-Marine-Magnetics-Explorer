# BlueOS Marine Magnetics Explorer

BlueOS extension for a **Marine Magnetics Explorer** towed magnetometer. It:

- Opens a user-selected **USB serial** port and baud rate (persisted in `state.json` on the logs volume).
- Captures the **towfish serial number** by sending the Explorer `I` command on connect, then starts cycling at the persisted **sample rate** (`0` Off, `1` 4 Hz, `2` 2 Hz, `3` 1 Hz; defaults to 4 Hz).
- Shows the **last 10 raw sentences** in the web UI.
- Polls **GPS / vehicle state** from [Mavlink2Rest](http://host.docker.internal/mavlink2rest/) — `GLOBAL_POSITION_INT` (position), `VFR_HUD` (groundspeed + AHRS heading, used for layback bearing and synthetic NMEA), and `GPS_RAW_INT` (fix type, satellites, HDOP/VDOP).
- Appends **CSV** logs under `/app/logs` with a new file per connection: `explorer_YYYYmmdd_HHMMSS.csv`.
- Writes per-session **raw text logs** to `/app/logs/RawMagYYYY-MM-DD.txt` and `RawGPSYYYY-MM-DD.txt`. The same process writes both files; for every received magnetometer sentence we append the raw mag line and a synthesised NMEA block (`$GPRMC`, `$GPGGA`, `$GPGSA`, `$GPGSV`) so the two files share the same row count and timestamps.
- Logs both the **vessel position** (`vessel_lat`, `vessel_lon`, `vessel_alt_m`) and the **estimated towfish position** (`towfish_lat`, `towfish_lon`) from layback offsets and AHRS heading.
- Computes a rolling **moving average** over a user-selectable window (4 / 10 / 15 / 30 / 60 / 120 s) and the **deviation** (`field_nT − moving_avg`). Triggers a UI/widget **alarm flash** + CSV `alarm` flag when `|deviation| > alarm_threshold_nt`.
- Sends **NAMED_VALUE_FLOAT** messages (total field, signal, depth, quality, deviation, alarm) to the autopilot log via Mavlink2Rest.
- Lets the operator **sync the magnetometer's clock** to current UTC (system clock / GPS-disciplined) via the Explorer `T` command.

The UI (Vue 2 + Vuetify) and fonts/scripts are **vendored in the Docker image** at build time so the vehicle does not need internet at runtime.

Pushes to `main` run the **Deploy BlueOS Extension** workflow and publish the Docker image when secrets are configured.

## Cockpit widget (line graph)

A standalone, framework-free line-chart page for embedding the live magnetic field trend in a Cockpit iframe is served at:

- `/widget` — light theme (default)
- `/widget?theme=dark` — dark theme
- `/widget?samples=600` — override rolling window size (10–5000)

The widget consumes `/api/events` (SSE) and `/api/status`, has no Vue/Vuetify dependency, and fills its iframe. Use it as the iframe URL of a Cockpit custom widget, e.g. `http://blueos.local/extensionv2/blueos-marine-magnetics-explorer/widget`.

## Layback position

The UI includes **Layback X** and **Layback Y** settings (meters), persisted in `state.json`.

- `layback_x_m`: positive is starboard, negative is port.
- `layback_y_m`: positive is forward, negative is behind the vehicle.

The extension estimates motion bearing from consecutive GPS fixes, rotates the X/Y offsets into local east/north meters, and writes `vessel_bearing_deg`, `towfish_lat`, `towfish_lon`, `layback_x_m`, and `layback_y_m` into each CSV row alongside the vessel position. If bearing is not available yet, the towfish lat/lon fall back to the current vessel GPS position.

## BlueOS install

1. Extensions Manager → **+ Manual install** (or equivalent).
2. Use image **`vshie/blueos-marine-magnetics-explorer`** and tag **`main`** (after CI publishes it).
3. Apply **Custom settings** (match the extension `LABEL permissions`):

```json
{
  "ExposedPorts": {
    "9091/tcp": {}
  },
  "HostConfig": {
    "Binds": [
      "/usr/blueos/extensions/marine-magnetics-explorer:/app/logs",
      "/dev:/dev"
    ],
    "ExtraHosts": ["host.docker.internal:host-gateway"],
    "PortBindings": {
      "9091/tcp": [
        {
          "HostPort": ""
        }
      ]
    },
    "Privileged": true
  }
}
```

Open the extension UI on the host port BlueOS assigns for **9091/tcp** (or the URL shown in Extensions).

## GitHub Actions / Docker Hub

The workflow uses [Deploy-BlueOS-Extension](https://github.com/BlueOS-community/Deploy-BlueOS-Extension).

**Secrets** (repository):

- `DOCKER_USERNAME`
- `DOCKER_PASSWORD`

**Variables** (optional; defaults shown in workflow):

- `IMAGE_NAME` — default `blueos-marine-magnetics-explorer`
- `MY_NAME`, `MY_EMAIL`, `ORG_NAME`, `ORG_EMAIL`

The first workflow run may fail until Docker Hub credentials are configured.

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PORT` | `9091` | HTTP listen port |
| `LOG_DIR` | `/app/logs` | Logs and `state.json` |
| `MAVLINK2REST_BASE` | `http://host.docker.internal/mavlink2rest/mavlink` | Mavlink2Rest HTTP prefix |
| `MAVLINK_VEHICLE_ID` | `1` | Vehicle id if `/vehicles` discovery fails |

## NAMED_VALUE_FLOAT names

| Name | Value |
|------|--------|
| `MAG_NT` | Total field (nT) |
| `MAG_SIG` | Signal strength |
| `MAG_DEPTH` | Depth (m), `0` if absent |
| `MAG_QUAL` | Quality / confidence, `0` if absent |
| `MAG_DEV` | Deviation from the moving average (nT) |
| `MAG_ALRM` | Alarm flag, `1.0` while `|deviation| > alarm_threshold_nt`, else `0.0` |

## Settings (persisted in `state.json`)

| Key | Default | Description |
|-----|---------|-------------|
| `port` | `""` | Serial device path (e.g. `/dev/ttyUSB0`) |
| `baud_rate` | `9600` | One of 1200, 2400, 4800, 9600, 19200 |
| `layback_x_m` | `0.0` | Lateral offset, +starboard / -port |
| `layback_y_m` | `-5.0` | Longitudinal offset, +forward / -behind |
| `sample_rate` | `"1"` | Explorer cycle command: `0` off, `1` 4 Hz, `2` 2 Hz, `3` 1 Hz |
| `avg_window_s` | `15` | Moving-average window in seconds (one of 4 / 10 / 15 / 30 / 60 / 120) |
| `alarm_threshold_nt` | `4.0` | Absolute deviation (nT) above which the alarm fires |

## CSV columns (added by this version)

`moving_avg_nt`, `deviation_nt`, `avg_window_s`, `alarm` are appended after `layback_y_m` and before `raw_sentence` so existing parsers that key off the header row continue to work.

## Raw per-session text files

For each connection session the extension also creates / appends to:

- `RawMagYYYY-MM-DD.txt` — one raw Explorer sentence per line, byte-for-byte.
- `RawGPSYYYY-MM-DD.txt` — one synthesised NMEA block (`$GPRMC` / `$GPGGA` / `$GPGSA` / `$GPGSV`) per mag sample, separated by blank lines. Speed comes from `VFR_HUD.groundspeed`, course from `VFR_HUD.heading`, sat count + HDOP from `GPS_RAW_INT`.

Both files are listed alongside CSVs on the **Logs** tab and exposed at `/api/logs/<name>` for download.

## HTTP API additions

| Method | Path | Body | Description |
|--------|------|------|-------------|
| POST | `/api/explorer/sync-time` | (none) | Sends `T` + 11-digit `JJJYYHHMMSS` (UTC). Refuses if there is no fresh GPS fix. |
| POST | `/api/avg-window` | `{"window_s": 15}` | Updates and persists the moving-average window. |
| POST | `/api/explorer/command` | `{"command": "cycle_4hz"}` | Existing route now accepts `cycle_off`, `cycle_4hz`, `cycle_2hz`, `cycle_1hz`, `autotune_on`, `autotune_off`. Cycle commands also persist `sample_rate`. |

## Local development (optional)

```bash
pip install flask==2.0.1 pyserial==3.5 requests flask-cors waitress
export LOG_DIR=./logs
mkdir -p logs static/vendor && cp -r /path/to/vendored/vendor/* static/vendor/  # or build Docker image
python app/main.py
```

For a full UI offline, build the Docker image (requires network during `docker build` only).

## License

MIT — see [LICENSE](LICENSE).

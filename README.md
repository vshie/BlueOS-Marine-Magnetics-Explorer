# BlueOS Marine Magnetics Explorer

BlueOS extension for a **Marine Magnetics Explorer** towed magnetometer. It:

- Opens a user-selected **USB serial** port and baud rate (persisted in `state.json` on the logs volume).
- Shows the **last 10 raw sentences** in the web UI.
- Polls **GPS** from [Mavlink2Rest](http://host.docker.internal/mavlink2rest/) (`GLOBAL_POSITION_INT`).
- Appends **CSV** logs under `/app/logs` with a new file per connection: `explorer_YYYYmmdd_HHMMSS.csv`.
- Sends **NAMED_VALUE_FLOAT** messages (total field, signal, depth, quality) to the autopilot log via Mavlink2Rest.

The UI (Vue 2 + Vuetify) and fonts/scripts are **vendored in the Docker image** at build time so the vehicle does not need internet at runtime.

Pushes to `main` run the **Deploy BlueOS Extension** workflow and publish the Docker image when secrets are configured.

## Simulation mode (no hardware)

Use the **Simulation** button in the UI (or `POST /api/connect` with JSON `{"simulate": true}`). The extension opens a CSV log named `explorer_YYYYmmdd_HHMMSS_sim.csv`, emits Explorer-format lines at ~4 Hz, parses them, posts **NAMED_VALUE_FLOAT** to Mavlink2Rest the same way as a real serial link, and still polls GPS from the vehicle if Mavlink2Rest is reachable. **Disconnect** stops simulation and closes the log.

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

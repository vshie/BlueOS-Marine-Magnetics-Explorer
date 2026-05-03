FROM python:3.11-slim-bullseye

RUN apt-get update && apt-get install -y \
    python3-serial curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN mkdir -p /app/logs && chmod 777 /app/logs

COPY app/ .

# Vendor frontend assets at build time (offline at runtime)
RUN mkdir -p static/vendor/js static/vendor/css static/vendor/fonts \
    && curl -fsSL -o static/vendor/js/vue.min.js \
        "https://cdn.jsdelivr.net/npm/vue@2.7.16/dist/vue.min.js" \
    && curl -fsSL -o static/vendor/js/vuetify.min.js \
        "https://cdn.jsdelivr.net/npm/vuetify@2.7.2/dist/vuetify.min.js" \
    && curl -fsSL -o static/vendor/js/axios.min.js \
        "https://cdn.jsdelivr.net/npm/axios@1.7.9/dist/axios.min.js" \
    && curl -fsSL -o static/vendor/css/vuetify.min.css \
        "https://cdn.jsdelivr.net/npm/vuetify@2.7.2/dist/vuetify.min.css" \
    && curl -fsSL -o static/vendor/css/materialdesignicons.min.css \
        "https://cdn.jsdelivr.net/npm/@mdi/font@7.4.47/css/materialdesignicons.min.css" \
    && curl -fsSL -o static/vendor/fonts/materialdesignicons-webfont.woff2 \
        "https://cdn.jsdelivr.net/npm/@mdi/font@7.4.47/fonts/materialdesignicons-webfont.woff2" \
    && curl -fsSL -o static/vendor/fonts/roboto-400.woff2 \
        "https://cdn.jsdelivr.net/fontsource/fonts/roboto@latest/latin-400-normal.woff2" \
    && curl -fsSL -o static/vendor/fonts/roboto-500.woff2 \
        "https://cdn.jsdelivr.net/fontsource/fonts/roboto@latest/latin-500-normal.woff2" \
    && curl -fsSL -o static/vendor/fonts/roboto-700.woff2 \
        "https://cdn.jsdelivr.net/fontsource/fonts/roboto@latest/latin-700-normal.woff2" \
    && printf '%s\n' \
        '@font-face{font-family:"Roboto";font-weight:400;font-style:normal;src:url("../fonts/roboto-400.woff2") format("woff2")}' \
        '@font-face{font-family:"Roboto";font-weight:500;font-style:normal;src:url("../fonts/roboto-500.woff2") format("woff2")}' \
        '@font-face{font-family:"Roboto";font-weight:700;font-style:normal;src:url("../fonts/roboto-700.woff2") format("woff2")}' \
        > static/vendor/css/roboto.css

RUN pip install --no-cache-dir flask==2.0.1 && \
    pip install --no-cache-dir pyserial==3.5 && \
    pip install --no-cache-dir requests==2.28.1 && \
    pip install --no-cache-dir Werkzeug==2.0.3 && \
    pip install --no-cache-dir Jinja2==3.0.3 && \
    pip install --no-cache-dir MarkupSafe==2.0.1 && \
    pip install --no-cache-dir itsdangerous==2.0.1 && \
    pip install --no-cache-dir flask-cors==3.0.10 && \
    pip install --no-cache-dir waitress==2.1.2

ENV PYTHONUNBUFFERED=1
ENV FLASK_APP=main.py
ENV PORT=9091

EXPOSE 9091

LABEL org.blueos.type="tool"
LABEL org.blueos.version="1.0.0"
LABEL org.blueos.requirements="core >= 1.1"
LABEL org.blueos.name="Marine Magnetics Explorer"
LABEL org.blueos.description="Explorer magnetometer serial logging with GPS and NAMED_VALUE_FLOAT to autopilot"
LABEL org.blueos.icon="mdi-magnet"
LABEL org.blueos.category="Data Collection"
LABEL org.blueos.order="20"

LABEL permissions='\
{\
 "ExposedPorts": {\
  "9091/tcp": {}\
 },\
 "HostConfig": {\
  "Binds": [\
   "/usr/blueos/extensions/marine-magnetics-explorer:/app/logs",\
   "/dev:/dev"\
  ],\
  "ExtraHosts": ["host.docker.internal:host-gateway"],\
  "PortBindings": {\
   "9091/tcp": [\
    {\
     "HostPort": ""\
    }\
   ]\
  },\
  "Privileged": true\
 }\
}'

LABEL org.blueos.authors='[{"name":"vshie","email":"vshie@users.noreply.github.com"}]'
LABEL org.blueos.company='{"about":"Marine Magnetics Explorer for BlueOS","name":"Blue Robotics","email":"support@bluerobotics.com"}'
LABEL org.blueos.readme=""
LABEL org.blueos.links='{"source":"https://github.com/vshie/BlueOS-Marine-Magnetics-Explorer"}'

CMD ["python", "-u", "main.py"]

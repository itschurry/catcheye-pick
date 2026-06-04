# catcheye-pick

RealSense D455 Isaac Sim 영상 수신용 CatchEye Pick 앱이다.

이 브랜치는 `codex/realsense-d455-sim` 기준이다. Isaac Sim 서버가 송출하는 D455 RGB/Depth MJPEG 스트림을 GStreamer로 받아 WebSocket으로 다시 송출한다.

## 설치

```bash
git submodule update --init --recursive
docker compose -f docker/docker-compose.dev.yml run --rm catcheye-pick-dev
```

`third_party/librealsense`는 Intel RealSense SDK submodule이다.

컨테이너 안에서 빌드:

```bash
./scripts/cmake.sh build
```

## 실행

Isaac Sim D455 영상 수신 후 WebSocket 송출:

```bash
./scripts/run-d455-sim.sh
```

기존 GStreamer RGB viewer-only:

```bash
./scripts/run-rgb.sh
```

NCNN detection + WebSocket:

```bash
./scripts/run-rgb-ncnn.sh
```

Hailo detection + WebSocket:

```bash
./scripts/run-rgb-hailo.sh
```

직접 실행:

```bash
./bin/catcheye-pick \
  --ws 8080 \
  --viewer-only \
  --input-source camera \
  --camera-backend isaacsim \
  --camera-pipeline "souphttpsrc location=http://210.120.123.164:8080/color.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720" \
  --depth-pipeline "souphttpsrc location=http://210.120.123.164:8080/depth.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720"
```

## 주요 옵션

| 옵션 | 설명 |
| --- | --- |
| `--input-source camera\|image\|video` | 입력 형태, 현재 실행 구현은 `camera`만 있음 |
| `--camera-backend realsense\|isaacsim` | 카메라 수신 방식, 현재 실행 구현은 `isaacsim`만 있음 |
| `--camera-pipeline <pipe>` | `--camera-backend isaacsim`에서 쓰는 필수 color GStreamer 입력 |
| `--depth-pipeline <pipe>` | `--camera-backend isaacsim`에서 쓰는 depth MJPEG 입력 |
| `--viewer-only` | detection 없이 영상만 송출 |
| `--ws [port]` | WebSocket 송출 활성화, 기본 `8080` |
| `--http-port <port>` | HTTP API 포트, 기본 `8090` |
| `--detector ncnn\|hailo` | detection backend |
| `--hef <path>` | Hailo HEF 모델 경로 |
| `--metadata <path>` | metadata YAML 경로 |
| `--roi <path>` | person ROI config |
| `--pallet-roi <path>` | pallet ROI config |
| `--intrinsics <path>` | camera intrinsics JSON 경로, 기본 `config/intrinsics.json` |
| `--extrinsics <path>` | camera extrinsics JSON 경로, 기본 `config/extrinsics.json` |
| `--robot-calibration <path>` | robot calibration config |

## HTTP API

| Method | Path | 설명 |
| --- | --- | --- |
| `GET` | `/api/device-info` | 앱 식별 정보 |
| `GET` | `/api/camera/intrinsics` | camera intrinsics JSON |
| `GET` | `/api/camera/extrinsics` | camera extrinsics JSON |
| `GET/PUT` | `/api/roi` | person ROI |
| `GET/PUT` | `/api/pallet-roi` | pallet ROI |
| `GET/PUT` | `/api/robot-calibration` | robot calibration |

## 디렉터리 구조

```text
.
├── CMakeLists.txt
├── cmake/
├── config/
│   ├── pallet_roi_cam_default.json
│   ├── intrinsics.json
│   ├── extrinsics.json
│   ├── robot_calibration.json
│   └── roi_cam_default.json
├── docker/
├── models/
├── scripts/
│   ├── cmake.sh
│   ├── run-d455-sim.sh
│   ├── run-rgb.sh
│   ├── run-rgb-ncnn.sh
│   └── run-rgb-hailo.sh
├── src/
│   ├── main.cpp
│   └── pick/
└── third_party/
    ├── catcheye-vision-sdk/
    └── librealsense/
```

## 빌드 산출물

```text
bin/catcheye-pick
bin/catcheye-pick-bin
```

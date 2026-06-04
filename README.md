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

실행 옵션 도움말:

```bash
./bin/catcheye-pick --help
```

Isaac Sim D455 영상 수신 후 WebSocket 송출:

```bash
./scripts/run-d455-sim.sh
```

기존 GStreamer RGB viewer-only:

```bash
./scripts/run-rgb.sh
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

## 실행 옵션

기본 형식:

```bash
./bin/catcheye-pick [입력 옵션] [부가 옵션]
```

입력 옵션:

- `--input-source <camera|image|video>`: 입력 형태를 지정한다. 현재 실행 구현은 `camera`만 있다.
- `--camera-backend <isaacsim|realsense>`: 카메라 수신 방식을 지정한다. 현재 실행 구현은 `isaacsim`만 있다.
- `--camera-pipeline <pipeline>`: `--camera-backend isaacsim`에서 쓰는 color GStreamer 입력이다.
- `--depth-pipeline <pipeline>`: `--camera-backend isaacsim`에서 쓰는 depth visualization GStreamer 입력이다.

부가 옵션:

- `-h`, `--help`: 실행 옵션 도움말을 출력한다.
- `--ws [port]`: WebSocket 결과 송출을 켠다. 포트를 생략하면 기본 `8080`을 쓴다.
- `--http-port <port>`: HTTP API 포트를 지정한다. 기본값은 `8090`이다.
- `--viewer-only`: 검출 없이 카메라 프레임만 송출한다. `--ws`가 필요하다.
- `--detector <hailo>`: detector 백엔드를 선택한다. 기본값은 `hailo`다.
- `--hef <path>`: Hailo 백엔드에서 사용할 HEF 모델 경로를 지정한다.
- `--metadata <path>`: 메타데이터 YAML 경로를 지정한다. 기본값은 `models/yolo26m_hailo_model/metadata.yaml`이다.
- `--roi <path>`: person ROI 설정 파일 경로를 덮어쓴다.
- `--pallet-roi <path>`: pallet ROI 설정 파일 경로를 덮어쓴다.
- `--intrinsics <path>`: camera intrinsics JSON 경로를 덮어쓴다.
- `--extrinsics <path>`: camera extrinsics JSON 경로를 덮어쓴다.
- `--robot-calibration <path>`: robot calibration 설정 파일 경로를 덮어쓴다.

제약 사항:

- `--input-source image`, `--input-source video`는 아직 구현되어 있지 않다.
- `--camera-backend realsense`는 아직 구현되어 있지 않다.
- `--camera-backend isaacsim`은 `--camera-pipeline`이 필요하다.
- `--viewer-only`는 `--ws`와 같이 써야 한다.
- `--viewer-only`에서는 모델과 메타데이터 인자를 쓰지 않는다.
- `--camera-pipeline`, `--depth-pipeline`은 `--camera-backend isaacsim`에서만 쓴다.

권장 실행 예시:

```bash
./bin/catcheye-pick --ws --viewer-only --camera-pipeline "<gst-color-pipeline>"
./bin/catcheye-pick --ws --detector hailo --hef models/yolo26m_hailo_model/yolo26m.hef --camera-pipeline "<gst-color-pipeline>"
```

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

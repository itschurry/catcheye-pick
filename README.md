# catcheye-pick

RealSense D455 Isaac Sim 영상 수신과 RGB-D 기반 pick 후보 생성을 위한 CatchEye Pick 앱이다.

Isaac Sim 서버가 송출하는 D455 RGB/Depth MJPEG 스트림을 GStreamer로 받고, 객체 CAD/USD 카탈로그와 Hailo 검출 bbox, depth frame을 조합해 camera 좌표계의 3D pick 후보를 WebSocket metadata로 송출한다.

지원 빌드 환경:

- `amd64`: Isaac Sim 연동용. 기본 빌드는 `libcamera`, `HailoRT` 없이 빌드한다.
- `arm64`: Raspberry Pi, NVIDIA Orin Nano 계열 하드웨어용. 기본 빌드는 `libcamera`, `HailoRT`를 켠다.

## 설치

```bash
git submodule update --init --recursive
./update_env.sh
docker compose -f docker/amd64/docker-compose.dev.yml run --rm catcheye-pick-develop-amd64
```

`third_party/librealsense`는 Intel RealSense SDK submodule이다.

amd64 호스트에서 arm64 컨테이너를 빌드하거나 실행하기 전에는 QEMU binfmt를 먼저 등록한다.
이 작업이 없으면 arm64 이미지 빌드 중 `exec /bin/bash: exec format error`가 난다.

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

arm64 하드웨어 컨테이너:

```bash
docker compose -f docker/arm64/docker-compose.dev.yml run --rm catcheye-pick-develop-arm64
```

호스트에서 실행 중인 컨테이너를 대상으로 빌드:

```bash
./scripts/cmake.sh build
```

arm64 컨테이너를 명시해서 빌드:

```bash
DOCKER_ARCH=arm64 ./scripts/cmake.sh build
```

`build/release-amd64/compile_commands.json` 또는 `build/release-arm64/compile_commands.json`은 컨테이너 경로 기준이다.
`build/compile_commands.json`은 macOS 호스트 경로 기준으로 변환된 파일이다.
프로젝트 루트에는 `compile_commands.json`을 만들지 않는다.

직접 CMake 설정:

```bash
cmake -S . -B build/release-amd64 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATCHEYE_PICK_ENABLE_LIBCAMERA=OFF \
  -DCATCHEYE_VISION_DETECTION_ENABLE_HAILO=OFF

cmake -S . -B build/release-arm64 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATCHEYE_PICK_ENABLE_LIBCAMERA=ON \
  -DCATCHEYE_VISION_DETECTION_ENABLE_HAILO=ON
```

주요 CMake 옵션:

- `CATCHEYE_PICK_ENABLE_LIBCAMERA`: `libcamera` 입력 백엔드 빌드 여부다. amd64 기본값은 `OFF`, arm64 기본값은 `ON`이다.
- `CATCHEYE_VISION_DETECTION_ENABLE_HAILO`: HailoRT detector 백엔드 빌드 여부다. amd64 기본값은 `OFF`, arm64 기본값은 `ON`이다.
- `CATCHEYE_PICK_BUILD_APP`: 실행 파일 빌드 여부다. 기본값은 `ON`이다.
- `CATCHEYE_PICK_BUILD_TESTS`: 앱 테스트 빌드 여부다. 기본값은 `OFF`다.

## 실행

실행 옵션 도움말:

```bash
./bin/catcheye-pick --help
```

Isaac Sim D455 영상 수신 후 WebSocket 송출:

```bash
./scripts/run-d455-sim.sh
```

Isaac Sim D455 RGB-D + Hailo 검출 + 3D pick 후보 송출:

```bash
./scripts/run-d455-sim-hailo.sh
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
  --depth-pipeline "souphttpsrc location=http://210.120.123.164:8080/depth.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720" \
  --depth-max-m 5.0
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
- `--depth-max-m <meters>`: depth frame 밝기 `255`가 의미하는 최대 거리다. detection에서 `--depth-pipeline`을 쓰면 필수다.
- `--depth-min-m <meters>`: depth sample에서 유효하다고 볼 최소 거리다. 기본값은 `0.05`다.

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
- `--objects <path>`: 객체 CAD/USD 카탈로그 JSON 경로를 덮어쓴다.

제약 사항:

- `--input-source image`, `--input-source video`는 아직 구현되어 있지 않다.
- `--camera-backend realsense`는 아직 구현되어 있지 않다.
- `--camera-backend isaacsim`은 `--camera-pipeline`이 필요하다.
- `--viewer-only`는 `--ws`와 같이 써야 한다.
- `--viewer-only`에서는 모델과 메타데이터 인자를 쓰지 않는다.
- `--camera-pipeline`, `--depth-pipeline`은 `--camera-backend isaacsim`에서만 쓴다.
- detection에서 `--depth-pipeline`을 쓰면 `--depth-max-m`도 같이 줘야 한다.
- 3D pick 후보는 `config/intrinsics.json`의 `width`, `height`, `fx`, `fy`, `cx`, `cy`와 depth frame 밝기값으로 계산한다.

권장 실행 예시:

```bash
./bin/catcheye-pick --ws --viewer-only --camera-pipeline "<gst-color-pipeline>"
./bin/catcheye-pick --ws --detector hailo --hef models/yolo26m_hailo_model/yolo26m.hef --camera-pipeline "<gst-color-pipeline>" --depth-pipeline "<gst-depth-pipeline>" --depth-max-m 5.0
```

## WebSocket metadata

`viewer_frame` metadata는 detection 실행에서 아래 값을 포함한다.

- `detections[].position`: bbox 중심 depth sample을 camera intrinsics로 투영한 camera 좌표다.
- `pick_candidates[]`: `position`이 있는 detection만 pick 후보로 변환한다.
- `pick_candidates[].object_id`: 현재 frame 안의 개별 객체 후보 id다. 같은 제품이 pallet 위에 여러 개 있으면 서로 다른 `object_id`를 갖는다.
- `pick_candidates[].product_id`: CAD/USD 카탈로그에 등록된 제품군 id다. 같은 제품 여러 개는 같은 `product_id`를 갖는다.
- `pick_candidates[].pose_camera`: camera 좌표계 기준 제품 pose다. 지금은 bbox+depth 기반 translation과 기본 rotation만 채운다.
- `pick_candidates[].pick_point_camera_m`: camera 좌표계 pick point다.
- `pick_candidates[].robot`: `config/robot_calibration.json`이 켜져 있고 confidence 조건을 넘을 때 R1/R2 좌표를 포함한다.
- `pose_estimates[]`: 외부 pose estimator가 `/api/pose-estimates`로 넣은 6D pose 결과다.
- `pose_estimates[].pick_point_camera_m`: `pose_camera * config/objects.json grasp.pick_point_object_m` 결과다.

## 객체 카탈로그

객체별 CAD/USD 기준 정보는 `config/objects.json`에 둔다.

현재 등록된 제품:

- product_id: `test_part`
- STEP: `assets/cad/test_part.step`
- USD: `assets/usd/test_part.usd`
- USD unit: `metersPerUnit = 0.001`
- object bbox: `0.32 x 0.22 x 0.01 m`
- grasp point: `[0.22466, 0.0951907, -0.005]`

`product_id`는 제품군 id다. `object_id`는 pallet 위에 실제로 놓인 개별 객체 instance id다.

## HTTP API

| Method | Path | 설명 |
| --- | --- | --- |
| `GET` | `/api/device-info` | 앱 식별 정보 |
| `GET` | `/api/camera/intrinsics` | camera intrinsics JSON |
| `GET` | `/api/camera/extrinsics` | camera extrinsics JSON |
| `GET` | `/api/objects` | object CAD/USD catalog JSON |
| `GET/PUT` | `/api/pose-estimates` | 외부 pose estimator 결과 |
| `GET/PUT` | `/api/roi` | person ROI |
| `GET/PUT` | `/api/pallet-roi` | pallet ROI |
| `GET/PUT` | `/api/robot-calibration` | robot calibration |

Pose estimator 결과 입력:

```bash
curl -X PUT http://127.0.0.1:8090/api/pose-estimates \
  -H 'Content-Type: application/json' \
  -d '{
    "estimates": [
      {
        "object_id": "test_part:1",
        "product_id": "test_part",
        "confidence": 0.92,
        "translation_m": [0.12, -0.03, 0.71],
        "rotation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]
      }
    ]
  }'
```

`rotation_quat_xyzw`는 정규화된 quaternion이어야 한다. `product_id`가 `config/objects.json`에 없거나 grasp point가 없으면 요청은 실패한다.

## 디렉터리 구조

```text
.
├── CMakeLists.txt
├── assets/
│   ├── cad/
│   │   └── test_part.step
│   └── usd/
│       └── test_part.usd
├── cmake/
├── config/
│   ├── pallet_roi_cam_default.json
│   ├── intrinsics.json
│   ├── extrinsics.json
│   ├── objects.json
│   ├── robot_calibration.json
│   └── roi_cam_default.json
├── docker/
│   ├── amd64/
│   └── arm64/
├── models/
├── scripts/
│   ├── cmake.sh
│   ├── run-d455-sim-hailo.sh
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

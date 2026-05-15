# CatchEye Pick

Raspberry Pi ARM64 환경을 대상으로 빌드하고 배포하는 picking 애플리케이션이다.
개발 PC에서는 Docker buildx/compose로 `linux/arm64` 개발 이미지를 빌드하고, 컨테이너 안에서 ARM64 타겟 빌드를 수행한다.
실행 대상 Raspberry Pi는 `Camera Module 3 + CubeEye + catcheye-pick` 구성을 기준으로 한다.

## 주요 기능

- Camera Module 3 입력 처리
- CubeEye depth/amplitude/rgb/pointcloud 프레임 입력 처리
- RGB-D source profile 선택: `rgb-cubeeye`, `rgb`, `cubeeye`
- WebSocket 송출
- viewer-only에서 Camera Module 3와 CubeEye stream 독립 송출
- viewer-only 모드에서 딥러닝 검출 비활성화
- NCNN/Hailo detector 선택
- CubeEye frame 선택 옵션
- Guard와 동일한 ROI HTTP API
- RGB-CubeEye offset HTTP API
- PointCloud ROI HTTP API
- Robot calibration HTTP API
- CubeEye S111D property HTTP API

## 기본 포트

| 기능 | 기본 포트 | 주소 |
| --- | ---: | --- |
| WebSocket | `8080` | `ws://<host>:8080/` |
| HTTP API | `8090` | `http://<host>:8090/api/` |

## 빠른 시작

개발 PC에서:

```bash
# x86_64 PC에서 arm64 컨테이너를 빌드/실행할 때 한 번만 필요하다.
docker run --privileged --rm tonistiigi/binfmt --install arm64

docker compose -f docker/docker-compose.dev.yml build
docker compose -f docker/docker-compose.dev.yml up -d

scripts/cmake.sh all

rsync -avz -e "ssh -p 2222" install/release-hailo/ user@arm-device:/opt/catcheye-pick/
```

ARM 장비에서:

```bash
cd /opt/catcheye-pick
./bin/catcheye-pick --viewer-only --ws
```

## 빌드/배포 개요

- Docker 이미지는 `linux/arm64` 플랫폼으로 빌드한다.
- 개발 PC에서는 QEMU/binfmt 기반 arm64 컨테이너 안에서 앱을 빌드한다.
- `cmake --install` 결과물을 Raspberry Pi의 `/opt/catcheye-pick/` 아래로 복사한다.
- 사용자는 `bin/catcheye-pick`을 실행한다.
- `bin/catcheye-pick`은 실행 진입점 역할을 하는 래퍼 스크립트다.
- `bin/catcheye-pick`은 필요한 라이브러리 경로를 설정한 뒤 실제 바이너리 `bin/catcheye-pick-bin`을 실행한다.
- `bin/catcheye-pick-bin`은 내부용 실행 파일이므로 직접 실행하지 않는다.

## 입력 모델

앱 내부 처리 기준은 RGB-D frame이다.
현재 구현된 source profile은 아래 세 가지다.

| profile | color 입력 | depth/pointcloud 입력 | 용도 |
| --- | --- | --- | --- |
| `rgb-cubeeye` | Camera Module 3 | CubeEye | 실제 picking 기본 구성 |
| `rgb` | Camera Module 3 | 없음 | RGB detector / viewer 확인 |
| `cubeeye` | 없음 | CubeEye | CubeEye depth / pointcloud viewer 확인 |

`--camera-input`은 카메라 장비 이름이 아니라 source profile 선택값이다.
새 카메라 구현은 processor에 직접 붙이지 말고 RGB-D frame 경계에 맞춰 추가한다.

## CMake 스크립트

개발 PC에서는 `scripts/cmake.sh`를 실행한다.
이 스크립트는 `catcheye-pick-develop-raspbian` 컨테이너 안에서 CMake 명령을 실행한다.

```bash
scripts/cmake.sh configure
scripts/cmake.sh build
scripts/cmake.sh install
scripts/cmake.sh verify
scripts/cmake.sh compile-db
scripts/cmake.sh all
scripts/cmake.sh clean
```

기본 profile은 `release-hailo`다.

```bash
scripts/cmake.sh all debug
scripts/cmake.sh all release
scripts/cmake.sh all release-hailo
```

컨테이너 이름이나 작업 경로가 다르면 환경변수로 지정한다.

```bash
CATCHEYE_PICK_CONTAINER=catcheye-pick-develop-raspbian \
CATCHEYE_PICK_CONTAINER_WORKDIR=/home/user/catcheye-pick \
scripts/cmake.sh build
```

`scripts/cmake.sh build`는 컨테이너에서 생성된 `compile_commands.json`의 프로젝트 경로를 현재 맥 체크아웃 경로로 변환한 뒤 `build/compile_commands.json`과 루트 `compile_commands.json`에 연결한다.
이미 빌드된 profile의 compile DB만 다시 맞출 때는 아래처럼 실행한다.

```bash
scripts/cmake.sh compile-db
scripts/cmake.sh compile-db debug
scripts/cmake.sh compile-db release
scripts/cmake.sh compile-db release-hailo
```

컨테이너 작업 경로나 맥 체크아웃 경로가 기본값과 다르면 명시한다.

```bash
CATCHEYE_PICK_CONTAINER_WORKDIR=/home/user/catcheye-pick \
CATCHEYE_PICK_HOST_WORKDIR=/Users/chlee/catcheye-pick \
scripts/cmake.sh compile-db release-hailo
```

## 실행 옵션

```bash
./bin/catcheye-pick [옵션]
```

- `--help`: 도움말을 출력한다.
- `--version`: CubeEye SDK 버전을 출력한다.
- `--list-cubeeye`: 연결된 CubeEye camera source를 출력한다.
- `--detector <ncnn|hailo>`: detector 백엔드를 선택한다. 기본값은 `ncnn`이다.
- `--hef <path>`: Hailo detector용 HEF 경로를 지정한다.
- `--metadata <path>`: detector metadata YAML 경로를 지정한다.
- `--num-threads <count>`: NCNN detector thread 수를 지정한다. 기본값은 `2`다.
- `--viewer-only`: 선택한 camera input을 켜고 딥러닝 검출 없이 송출한다.
- `--ws [port]`: WebSocket 송출을 켠다. 포트를 생략하면 기본 포트 `8080`을 사용한다.
- `--http-port <port>`: HTTP API 포트를 지정한다. 기본값은 `8090`이다.
- `--camera-input <mode>`: RGB-D source profile을 지정한다. `rgb-cubeeye`, `rgb`, `cubeeye` 중 하나다. 기본값은 `rgb-cubeeye`다.
- `--camera-pipeline <pipeline>`: Camera Module 3 GStreamer pipeline을 덮어쓴다.
- `--roi <path>`: Person ROI config 경로를 지정한다.
- `--pallet-roi <path>`: Pallet ROI config 경로를 지정한다.
- `--rgb-cubeeye-offset <path>`: RGB-CubeEye offset config 경로를 지정한다. 기본값은 `config/rgb_cubeeye_offset.json`이다.
- `--pointcloud-roi <path>`: PointCloud X/Y/Z ROI config 경로를 지정한다. 기본값은 `config/pointcloud_roi.json`이다.
- `--robot-calibration <path>`: Robot calibration config 경로를 지정한다. 기본값은 `config/robot_calibration.json`이다.
- `--cubeeye-frames <list>`: CubeEye frame 목록을 지정한다. 기본값은 `depth,amplitude`다.
- `depth`와 `pointcloud`는 CubeEye SDK 제약으로 동시에 선택할 수 없다.
- `--cubeeye-camera-fps <fps>`: CubeEye S111D camera framerate를 지정한다. 허용값은 `7`, `15`, `30`이다.
- `--pointcloud-downsample <stride>`: pointcloud 송출 downsample 간격을 지정한다. 기본값은 `4`다.
- `--rtsp`: 지원하지 않는다.

## detector 실행 예시

NCNN:

```bash
./bin/catcheye-pick --camera-input rgb --detector ncnn \
  --ws \
  ./models/yolo26n_ncnn_model/model.ncnn.param \
  ./models/yolo26n_ncnn_model/model.ncnn.bin \
  ./models/yolo26n_ncnn_model/metadata.yaml
```

Hailo:

```bash
./bin/catcheye-pick --camera-input rgb --detector hailo \
  --ws \
  --hef ./models/yolo26m_hailo_model/yolo26m.hef \
  --metadata ./models/yolo26m_hailo_model/metadata.yaml
```

Camera Module 3 + CubeEye:

```bash
./bin/catcheye-pick --camera-input rgb-cubeeye --detector ncnn --ws --cubeeye-frames pointcloud
```

RGB↔CubeEye 오프셋 조정:

```bash
curl -X PUT http://localhost:8090/api/rgb-cubeeye-offset \
  -H 'Content-Type: application/json' \
  -d '{"u":0.00,"v":0.40}'
```

PointCloud ROI 조정:

```bash
curl -X PUT http://localhost:8090/api/pointcloud-roi \
  -H 'Content-Type: application/json' \
  -d '{"enabled":true,"apply_to_viewer":true,"min_x_m":-1.0,"max_x_m":1.0,"min_y_m":-1.0,"max_y_m":1.0,"min_z_m":0.0,"max_z_m":3.0}'
```

Robot calibration 조정:

```bash
curl -X PUT http://localhost:8090/api/robot-calibration \
  -H 'Content-Type: application/json' \
  -d '{"enabled":true,"min_confidence":0.5,"r1_tx_m":0.0,"r1_ty_m":0.0,"r1_tz_m":0.0,"r1_roll_deg":0.0,"r1_pitch_deg":0.0,"r1_yaw_deg":0.0,"r2_tx_m":0.0,"r2_ty_m":0.0,"r2_tz_m":0.0,"r2_roll_deg":0.0,"r2_pitch_deg":0.0,"r2_yaw_deg":0.0}'
```

## viewer-only 예시

기본 실행:

```bash
./bin/catcheye-pick --viewer-only --ws
```

Camera Module 3만:

```bash
./bin/catcheye-pick --viewer-only --ws --camera-input rgb
```

CubeEye만:

```bash
./bin/catcheye-pick --viewer-only --ws --camera-input cubeeye
```

CubeEye depth/amplitude:

```bash
./bin/catcheye-pick --viewer-only --ws --cubeeye-frames depth,amplitude
```

CubeEye camera fps 지정:

```bash
./bin/catcheye-pick --viewer-only --ws --cubeeye-camera-fps 15
```

CubeEye depth/amplitude/rgb:

```bash
./bin/catcheye-pick --viewer-only --ws --cubeeye-frames depth,amplitude,rgb
```

CubeEye pointcloud:

```bash
./bin/catcheye-pick --viewer-only --ws --cubeeye-frames pointcloud --pointcloud-downsample 4
```

Camera Module 3 pipeline 지정:

```bash
./bin/catcheye-pick --viewer-only --ws --camera-pipeline "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=10/1,format=NV12 ! videoflip method=rotate-180"
```

## HTTP API

- `GET /api/roi`
- `PUT /api/roi`
- `GET /api/pallet-roi`
- `PUT /api/pallet-roi`
- `GET /api/rgb-cubeeye-offset`
- `PUT /api/rgb-cubeeye-offset` body: `{"u": 0.00, "v": 0.40}`
- `GET /api/pointcloud-roi`
- `PUT /api/pointcloud-roi`
- `GET /api/robot-calibration`
- `PUT /api/robot-calibration`
- `GET /api/cubeeye/properties`
- `PUT /api/cubeeye/properties/{key}` body: `{"value": ...}`

PointCloud ROI body:

```json
{
  "enabled": true,
  "apply_to_viewer": true,
  "min_x_m": -1.0,
  "max_x_m": 1.0,
  "min_y_m": -1.0,
  "max_y_m": 1.0,
  "min_z_m": 0.0,
  "max_z_m": 3.0
}
```

Robot calibration body:

```json
{
  "enabled": true,
  "min_confidence": 0.5,
  "r1_tx_m": 0.0,
  "r1_ty_m": 0.0,
  "r1_tz_m": 0.0,
  "r1_roll_deg": 0.0,
  "r1_pitch_deg": 0.0,
  "r1_yaw_deg": 0.0,
  "r2_tx_m": 0.0,
  "r2_ty_m": 0.0,
  "r2_tz_m": 0.0,
  "r2_roll_deg": 0.0,
  "r2_pitch_deg": 0.0,
  "r2_yaw_deg": 0.0
}
```

CubeEye property 지원 항목:

- `framerate`: `7`, `15`, `30`
- `auto_exposure`: `true`, `false`
- `illumination`: `true`, `false`
- `depth_range_min`: `0..8192`
- `depth_range_max`: `0..8192`
- `amplitude_time_filter`: `true`, `false`
- `depth_average_median_filter`: `true`, `false`
- `depth_time_filter`: `true`, `false`
- `flying_pixel_remove_filter`: `true`, `false`
- `noise_filter1`: `true`, `false`
- `noise_filter2`: `true`, `false`
- `noise_filter3`: `true`, `false`
- `amplitude_threshold_min`: `0..`
- `amplitude_threshold_max`: `0..`
- `amplitude_time_spatial_threshold`: number
- `amplitude_time_temporal_threshold`: number
- `depth_average_median_max_n`: `0..`
- `depth_offset`: `0..`
- `depth_time_spatial_threshold`: number
- `depth_time_temporal_threshold`: number
- `flying_pixel_remove_threshold`: `0..`
- `integration_time`: `0..`
- `motion_blur_frequency`: `0..`
- `motion_blur_threshold`: `0..`
- `motion_blur_threshold2`: `0..`
- `scattering_threshold`: `0..`

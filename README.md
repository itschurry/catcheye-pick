# CatchEye Pick

Raspberry Pi ARM64 환경을 대상으로 빌드하고 배포하는 picking 애플리케이션이다.
개발 PC에서는 Docker buildx/compose로 `linux/arm64` 개발 이미지를 빌드하고, 컨테이너 안에서 ARM64 타겟 빌드를 수행한다.
실행 대상 Raspberry Pi는 `Camera Module 3 + CubeEye + catcheye-pick` 구성을 기준으로 한다.

## 주요 기능

- Camera Module 3 입력 처리
- CubeEye depth/amplitude/rgb/pointcloud 프레임 입력 처리
- `rgb-cubeeye`, `rgb`, `cubeeye` camera input 선택
- WebSocket 송출
- viewer-only에서 Camera Module 3와 CubeEye stream 독립 송출
- viewer-only 모드에서 딥러닝 검출 비활성화
- NCNN/Hailo detector 선택
- CubeEye frame 선택 옵션
- Guard와 동일한 ROI HTTP API
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
docker compose -f docker/docker-compose.dev.yml run --rm catcheye-pick-dev bash

cmake -S /home/user/catcheye-pick -B /home/user/catcheye-pick/build/release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release

cmake --build /home/user/catcheye-pick/build/release -j$(nproc)
cmake --install /home/user/catcheye-pick/build/release --prefix /home/user/catcheye-pick/install/release

rsync -av /home/user/catcheye-pick/install/release/ user@arm-device:/opt/catcheye-pick/
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
- `--camera-input <mode>`: camera input을 지정한다. `rgb-cubeeye`, `rgb`, `cubeeye` 중 하나다. 기본값은 `rgb-cubeeye`다.
- `--camera-pipeline <pipeline>`: Camera Module 3 GStreamer pipeline을 덮어쓴다.
- `--roi <path>`: Person ROI config 경로를 지정한다.
- `--pallet-roi <path>`: Pallet ROI config 경로를 지정한다.
- `--cubeeye-frames <list>`: CubeEye frame 목록을 지정한다. 기본값은 `depth,amplitude`다.
- `depth`와 `pointcloud`는 CubeEye SDK 제약으로 동시에 선택할 수 없다.
- `--cubeeye-camera-fps <fps>`: CubeEye S111D camera framerate를 지정한다. 허용값은 `7`, `15`, `30`이다.
- `--pointcloud-downsample <stride>`: pointcloud 송출 downsample 간격을 지정한다. 기본값은 `4`다.

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
  --hef ./models/model.hef \
  --metadata ./models/metadata.yaml
```

Camera Module 3 + CubeEye:

```bash
./bin/catcheye-pick --camera-input rgb-cubeeye --detector ncnn --ws --cubeeye-frames pointcloud
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
- `GET /api/cubeeye/properties`
- `PUT /api/cubeeye/properties/{key}` body: `{"value": ...}`

CubeEye property v1 지원 항목:

- `framerate`: `7`, `15`, `30`
- `auto_exposure`: `true`, `false`
- `illumination`: `true`, `false`
- `depth_range_min`: `0..8192`
- `depth_range_max`: `0..8192`

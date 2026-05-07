# catcheye-pick

`catcheye-guard`와 같은 CMake/Docker 구조로 가는 picking 앱이야.

## 구조

- `src/pick`: catcheye-pick 앱 코드
- `third_party/catcheye-vision-sdk`: vision-sdk 서브모듈
- `third_party/CubeEye2.0-sdk`: CubeEye 3D 카메라 SDK
- `docker`: Raspberry Pi arm64 개발 컨테이너

CubeEye SDK는 업체 제공 압축파일을 `third_party/CubeEye2.0-sdk`에 풀어둔 상태로 사용해.
별도 설치는 하지 않고, install 단계에서 필요한 헤더와 런타임 라이브러리만 같이 복사해.

## 개발 컨테이너

```bash
./update_env.sh
docker compose -f docker/docker-compose.dev.yml up -d --build
docker compose -f docker/docker-compose.dev.yml exec catcheye-pick-dev bash
```

## 빌드

```bash
cmake -S . -B build/release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
```

## 실행

```bash
./build/release/catcheye-pick-bin --help
./build/release/catcheye-pick-bin --version
./build/release/catcheye-pick-bin --list-cubeye
```

# Calibration Boards

RGB intrinsic과 RGB-CubeEye extrinsic 캘리브레이션용 인쇄 보드다.
파일은 SVG라서 벡터 그대로 출력해야 한다. 프린터 옵션은 배율 `100%`로 고정한다.

## 파일

| 파일 | 용도 | 용지 | 패턴 | OpenCV patternSize | squareSize |
| --- | --- | --- | --- | --- | --- |
| `rgb_intrinsic_a4_checkerboard.svg` | RGB intrinsic | A4 landscape | 10 x 7 squares | `9x6` | `0.020` m |
| `rgb_cubeeye_extrinsic_a3_checkerboard.svg` | RGB-CubeEye extrinsic | A3 landscape | 9 x 6 squares | `8x5` | `0.040` m |

## RGB intrinsic 보드

- Camera Module 3 intrinsic 추출용이다.
- A4에 20 mm square를 넣었다.
- 화면 구석까지 패턴을 여러 각도로 채워서 촬영한다.
- 휘어진 종이는 쓰지 않는다. 폼보드나 아크릴에 붙인다.

## RGB-CubeEye extrinsic 보드

- RGB는 checkerboard corner를 보고, CubeEye는 같은 보드의 평면을 본다.
- A3에 40 mm square를 넣었다. A3 안에서 CubeEye plane 크기를 최대한 키운 크기다.
- 반드시 단단하고 평평한 판에 붙인다. 보드가 휘면 extrinsic 값이 틀어진다.
- 검은색은 무광으로 출력한다. 반사지는 쓰지 않는다.

## 출력 확인

출력 후 자로 한 칸 크기를 확인한다.

```bash
# RGB intrinsic
squareSize = 20 mm
patternSize = 9x6

# RGB-CubeEye extrinsic
squareSize = 40 mm
patternSize = 8x5
```

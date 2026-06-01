# 2026-01-BeamBalancing 프로젝트 문서

최초 작성: 2026-04-06 19:31:41 KST  
최종 수정: 2026-05-08 KST

## 1) 문서 목적
본 문서는 STM32F411RE Nucleo-64 기반 2026-01-BeamBalancing 프로젝트의 현재 동작 정리 문서임.

- 다중 채널 ADC(3채널) + TIM3 외부 트리거 샘플링
- TIM4 PWM(50Hz) 기반 SG-90 제어
- UART 명령 인터페이스 기반 모드 선택/런타임 제어
- **TIM1 Input Capture 기반 HC-SR04 초음파 거리 측정 (100Hz, 인터럽트)**
- **USART2 RX 인터럽트 + 링버퍼 방식으로 STOP 명령 유실 문제 수정**

## 배선도 작성용 연결표 (핵심)

아래 표를 기준으로 실제 배선도를 작성하면 됩니다.

| 대상 | 센서/모듈 핀 | MCU 핀 | 보드 핀명 | 비고 |
|---|---|---|---|---|
| Optical distance sensor | AOUT | PA0 (ADC1_IN0, Rank1) | A0 | 주 측정 입력 |
| Optical distance sensor (옵션) | AOUT2 | PA1 (ADC1_IN1, Rank2) | A1 | 보조 입력 |
| Optical distance sensor (옵션) | AOUT3 | PA4 (ADC1_IN4, Rank3) | A2 | 보조 입력 |
| SG-90 서보 | SIG | PB6 (TIM4_CH1) | D10 | PWM 50Hz |
| HC-SR04 | TRIG | PA6 (GPIO Out) | D12 | 10us 펄스 출력 |
| HC-SR04 | ECHO | PA8 (TIM1_CH1 Input Capture) | D7 | 입력 캡처 |
| UART 로그/명령 | TX | PA2 (USART2_TX) | D1 | PC 수신 측 |
| UART 로그/명령 | RX | PA3 (USART2_RX) | D0 | PC 송신 측 |

배선 전원/접지 규칙:
- SG-90 VCC는 외부 5V 권장, SG-90 GND와 보드 GND는 반드시 공통 접지
- HC-SR04 VCC는 5V 사용 가능, HC-SR04 GND와 보드 GND 공통 접지
- HC-SR04 ECHO(5V)는 PA8(3.3V 입력)에 직접 연결하지 말고 저항 분배기 사용 권장 (예: 1k/2k)

배선도 작성 체크리스트:
- ADC0(주센서)는 반드시 PA0/A0에 연결
- SG-90 신호선은 PB6/D10에 연결
- HC-SR04는 TRIG=D12, ECHO=D7로 분리 연결
- 모든 모듈의 GND를 단일 공통 GND로 묶기

회로도 툴 작업 파일:
- `schematic_netlist.csv`: 회로도 툴에 옮겨 그리기 위한 네트 연결표
- `schematic_parts.md`: 부품 ID/레벨변환/전원 규칙 정리

## 2) 적용 파일
- Core/Src/main.c
- Core/Src/stm32f4xx_hal_msp.c
- Core/Src/stm32f4xx_it.c
- Core/Inc/main.h
- 2026-01-BeamBalancing.ioc
- cmake/gcc-arm-none-eabi.cmake

## 3) 현재 핵심 기능

### 3.1 부팅 모드 선택
부팅 직후 UART에서 모드 선택 수행.

- `1`: ADC 모드
- `2`: Servo 모드

출력 예:
- `=== Mode Select ===`
- `1: ADC Mode   - 3ch raw data CSV output`
- `2: Servo Mode - command interface (A, CMIN, CMAX, STATUS, HELP)`

### 3.2 ADC 모드 명령 인터페이스
ADC 모드 진입 후 `Input>` 프롬프트에서 아래 명령 지원.

- `START`: CSV 스트리밍 출력 활성화
- `STOP`: CSV 스트리밍 출력 비활성화
- `STATUS`: ADC 상태 출력
- `HELP`: 도움말 출력

CSV 출력 형식:
- `raw0,raw1,raw2`

### 3.3 Servo 모드 명령 인터페이스
Servo 모드 진입 후 `Input>` 프롬프트에서 아래 명령 지원.

- `A<deg>`: 각도 설정(`A0` ~ `A180`)
- `CMIN:<us>`: 0도 펄스폭 설정(`500`~`3000`)
- `CMAX:<us>`: 180도 펄스폭 설정(`500`~`3000`)
- `STATUS`: 현재 각도/펄스폭 상태 출력
- `HELP`: 도움말 출력

서보 초기값:
- 초기 각도: `90 deg`
- 기본 캘리브레이션: `CMIN=500us`, `CMAX=2500us`

## 4) ADC 샘플링 구성

### 4.1 ADC 설정
- ADC1, 12-bit, Scan mode enabled
- External trigger: TIM3 TRGO (rising edge)
- Number of conversion: 3
- EOC selection: single conversion

채널 순서:
- Rank 1: ADC_CHANNEL_0 (PA0)
- Rank 2: ADC_CHANNEL_1 (PA1)
- Rank 3: ADC_CHANNEL_4 (PA4)

### 4.2 샘플링 주기(TIM3)
- Prescaler = 8399
- Period = 99
- TRGO = Update event

계산:
- TIM3 clock = 84MHz
- Counter clock = 84MHz / (8399 + 1) = 10kHz
- Update period = (99 + 1) / 10kHz = 10ms
- Sampling rate = 100Hz

## 6) Servo PWM 구성

### 6.1 TIM4 PWM
- TIM4 CH1 (PB6)
- Prescaler = 83
- Period = 19999
- PWM frequency = 50Hz(20ms)

계산:
- TIM4 clock = 84MHz
- Counter clock = 84MHz / (83 + 1) = 1MHz (1 tick = 1us)
- Period = 20000us = 20ms = 50Hz

### 6.2 각도 -> 펄스폭 변환
서보 펄스폭 계산식은 아래와 같음.

- `pulse_us = CMIN + ((angle * (CMAX - CMIN)) / 180)`

범위:
- angle: 0~180
- CMIN/CMAX: 500~3000us (단, CMIN < CMAX)

## 7) HC-SR04 초음파 거리 측정 (TIM1 Input Capture)

### 7.1 개요
TIM1을 이용하여 HC-SR04 초음파 센서를 100Hz 주기로 연속 측정한다.  
측정 결과는 `g_hcsr04_dist_cm` (float) 변수에 인터럽트 내에서 갱신된다.

### 7.2 핀 배정

| 신호 | MCU 핀 | 방향 | 설정 |
|------|--------|------|------|
| TRIG | PA6 | Output | GPIO_Output, Push-Pull |
| ECHO | PA8 | Input | TIM1_CH1, AF1, Pull-Down |

### 7.3 TIM1 타이머 설정

| 파라미터 | 값 | 비고 |
|----------|----|------|
| 클럭 소스 | APB2 TIM clock (84MHz) | |
| Prescaler | 83 | 84MHz / 84 = **1MHz** (1 tick = 1µs) |
| Period (ARR) | 9999 | 10000 tick = **10ms = 100Hz** |
| CH1 모드 | Input Capture | ECHO 엣지 시각 측정 |
| CH2 모드 | Output Compare TIMING | TRIG 10µs 펄스 종료 타이밍 |

### 7.4 측정 동작 원리 (인터럽트 기반)

HC-SR04는 10µs 이상의 TRIG 펄스를 보내면, ECHO 핀을 HIGH로 올렸다가 초음파가 반사되어 돌아오는 시간만큼 HIGH를 유지한 후 LOW로 내린다.  
거리는 ECHO HIGH 폭(µs)을 58로 나누면 cm 단위로 환산된다.

```
distance_cm = ECHO_width_us / 58.0
```

인터럽트 처리 흐름:

```
[TIM1 Update IRQ - 10ms마다]
  1. 이전 측정 미완료 시 → ECHO_IDLE 리셋
  2. TRIG = HIGH

[TIM1 CH2 OC IRQ - Update로부터 10µs 후]
  3. TRIG = LOW  (10µs 펄스 완료)

[TIM1 CH1 IC IRQ - ECHO 상승 엣지]
  4. 카운터값 저장 → g_echo_start_us
  5. 캡처 극성을 하강 엣지로 전환

[TIM1 CH1 IC IRQ - ECHO 하강 엣지]
  6. 카운터값 읽기 → echo_end
  7. 폭 계산: width = echo_end - g_echo_start_us
     (카운터 랩오버 시: width = ARR - start + end + 1)
  8. g_hcsr04_dist_cm = width / 58.0f
  9. 캡처 극성을 상승 엣지로 복원
```

### 7.5 카운터 랩오버 보정
ECHO 측정 도중 TIM1 카운터가 ARR(9999)에서 0으로 넘어가는 경우, 단순 뺄셈이 음수가 되므로 아래와 같이 보정한다.

```c
if (echo_end >= g_echo_start_us)
    width_us = echo_end - g_echo_start_us;
else
    width_us = (HCSR04_TIM_ARR - g_echo_start_us) + echo_end + 1U;
```

최대 측정 가능 거리:
- 10ms 내 왕복 가능 거리 = 10ms / 2 × 34300 cm/s ≈ **171 cm**

### 7.6 NVIC 우선순위

| IRQ | 우선순위 | 비고 |
|-----|---------|------|
| TIM1_UP_TIM10_IRQn | 2, 0 | Update + TRIG 생성 |
| TIM1_CC_IRQn | 2, 0 | CH1 IC + CH2 OC |
| ADC_IRQn | 0, 0 | ADC 변환 완료 |
| USART2_IRQn | 1, 0 | UART RX |

### 7.7 관련 전역 변수

| 변수 | 타입 | 설명 |
|------|------|------|
| `g_hcsr04_dist_cm` | `volatile float32_t` | 최신 측정 거리 (cm) |
| `g_echo_width_us` | `volatile uint32_t` | 최신 ECHO 폭 (µs) |
| `g_echo_start_us` | `volatile uint32_t` | ECHO 상승 엣지 캡처값 |
| `g_echo_state` | `volatile EchoState_t` | ECHO_IDLE / ECHO_RISING |

## 8) UART RX 인터럽트 방식 (링버퍼)

### 8.1 변경 배경
ADC 모드에서 CSV를 연속 출력하는 동안, UART 수신을 폴링(`HAL_UART_Receive` timeout=0)으로만 처리하면 `STOP` 등 다중 바이트 명령이 RX 오버런으로 유실되는 문제가 있었다.

### 8.2 해결 방법
`HAL_UART_Receive_IT`를 이용한 1바이트 인터럽트 수신 + 소프트웨어 링버퍼 방식으로 전환.

동작 흐름:
1. 부팅 시 `HAL_UART_Receive_IT` 1회 호출로 수신 시작
2. 바이트 수신마다 `HAL_UART_RxCpltCallback` 호출 → 링버퍼에 push → 즉시 재등록
3. `ProcessAdcInput` / `ProcessServoInput`에서 링버퍼 pop으로 명령 파싱
4. UART 에러 발생 시 `HAL_UART_ErrorCallback`에서 플래그 클리어 후 재등록

링버퍼 크기: 64바이트 (`UART_RX_RING_SIZE`)

## 9) 런타임 동작 개요

### 9.1 공통 초기화
1. GPIO/UART/ADC/TIM1/TIM3/TIM4 초기화
2. TIM4 PWM 시작
3. ADC 큐/서보 초기 상태 설정
4. TIM3 시작 (ADC 트리거 타이머)
5. UART RX 인터럽트 시작
6. ADC 인터럽트 시작
7. TRIG 핀 초기 LOW 설정
8. TIM1 CH1 IC 인터럽트 시작 (ECHO 캡처)
9. UART 모드 선택

### 9.2 메인 루프
- Servo 모드: UART 명령 처리(`A`, `CMIN`, `CMAX`, `STATUS`, `HELP`)
- ADC 모드: UART 명령 처리(`START`, `STOP`, `STATUS`, `HELP`) + 큐 pop/CSV 출력
- HC-SR04 거리 측정: 인터럽트 전용 (메인 루프 비점유), `g_hcsr04_dist_cm` 읽기만 하면 됨

## 10) UART 출력 예시

### 10.1 ADC 모드
- `Input> STATUS`
- `[ADC STATUS]`
- `stream    : ON`
- `sample    : 100 Hz`

### 10.2 Servo 모드
- `Input> A90`
- `Angle set: 90 deg (1500us)`
- `Input> CMIN:600`
- `CMIN = 600 us (CMAX = 2500 us)`

## 11) CubeMX(.ioc) 반영 상태
현재 .ioc는 코드와 일치하는 설정 상태임.

- ADC External Trigger: `TIM3 TRGO`
- TIM1: `Prescaler=83`, `Period=9999`, CH1 Input Capture
- TIM3: `Prescaler=8399`, `Period=99`, `MasterOutputTrigger=TIM_TRGO_UPDATE`
- TIM4 PWM: `CH1`, `Prescaler=83`, `Period=19999`
- NVIC: `USART2_IRQn(1,0)`, `TIM1_UP_TIM10_IRQn(2,0)`, `TIM1_CC_IRQn(2,0)` 활성화
- 핀 라벨 반영:
  - `PA0: ADC_CH0_IN0`
  - `PA1: ADC_CH1_IN1`
  - `PA4: ADC_CH2_IN4`
  - `PA6: HCSR04_TRIG`
  - `PA8: HCSR04_ECHO`
  - `PB6: SERVO_PWM_TIM4_CH1`

## 12) 핀맵 (STM32F411RE Nucleo-64)

| 기능 | MCU 핀 | 보드 핀명 | 주변장치/채널 | 비고 |
|---|---|---|---|---|
| ADC 입력 1 | PA0 | A0 | ADC1_IN0 (Rank 1) | 아날로그 입력 |
| ADC 입력 2 | PA1 | A1 | ADC1_IN1 (Rank 2) | 아날로그 입력 |
| ADC 입력 3 | PA4 | A2 | ADC1_IN4 (Rank 3) | 아날로그 입력 |
| HC-SR04 TRIG | PA6 | D12 | GPIO Output | 초음파 트리거 |
| HC-SR04 ECHO | PA8 | D7 | TIM1_CH1 (AF1) | 초음파 에코 |
| 서보 PWM 출력 | PB6 | D10 | TIM4_CH1 (AF2) | SG-90 신호선 |
| UART TX | PA2 | D1 | USART2_TX | 로그/CSV 출력 |
| UART RX | PA3 | D0 | USART2_RX | 명령 입력 |
| 사용자 LED | PA5 | D13 | GPIO Output | LD2 |
| 사용자 버튼 | PC13 | B1 | GPIO Input/EXTI | 기본 버튼 |

내부 연결(외부 핀 없음):
- TIM3 Update Event(TRGO) → ADC1 External Trigger
- TIM1 CH2 OC TIMING → TRIG 펄스 10µs 종료

## 13) 빌드
- 명령: `cmake --build build/Debug -j4`
- 상태: 정상 빌드 확인

## 14) 주의 사항
- SG-90 전원은 가능하면 외부 5V 사용 권장
- 외부 전원 사용 시 STM32 보드와 GND 공통 연결 필수
- CSV 출력은 UART 트래픽을 크게 증가시키므로 필요 시 `STOP`으로 중지
- `printf` float 출력은 코드 크기 증가 가능
- HC-SR04 ECHO 핀은 5V 출력이므로 PA8에 **전압 분배 회로(저항 분배)** 연결 권장  
  (예: 1kΩ + 2kΩ 분배로 5V → 3.3V 변환)
- HC-SR04 최대 측정 거리는 ARR=9999 기준 **약 171cm**;  
  더 긴 거리가 필요하면 ARR 값을 키우고 측정 주기를 낮출 것

## 1) 문서 목적
본 문서는 STM32F411RE Nucleo-64 기반 2026-01-BeamBalancing 프로젝트의 현재 동작 정리 문서임.

- 다중 채널 ADC(3채널) + TIM3 외부 트리거 샘플링
- TIM4 PWM(50Hz) 기반 SG-90 제어
- UART 명령 인터페이스 기반 모드 선택/런타임 제어

## 2) 적용 파일
- Core/Src/main.c
- Core/Src/stm32f4xx_hal_msp.c
- Core/Inc/main.h
- 2026-01-BeamBalancing.ioc
- cmake/gcc-arm-none-eabi.cmake

## 3) 현재 핵심 기능

### 3.1 부팅 모드 선택
부팅 직후 UART에서 모드 선택 수행.

- `1`: ADC 모드
- `2`: Servo 모드

출력 예:
- `=== Mode Select ===`
- `1: ADC Mode   - 3ch raw data CSV output`
- `2: Servo Mode - command interface (A, CMIN, CMAX, STATUS, HELP)`

### 3.2 ADC 모드 명령 인터페이스
ADC 모드 진입 후 `Input>` 프롬프트에서 아래 명령 지원.

- `START`: CSV 스트리밍 출력 활성화
- `STOP`: CSV 스트리밍 출력 비활성화
- `STATUS`: ADC 상태 출력
- `HELP`: 도움말 출력

CSV 출력 형식:
- `raw0,raw1,raw2`

### 3.3 Servo 모드 명령 인터페이스
Servo 모드 진입 후 `Input>` 프롬프트에서 아래 명령 지원.

- `A<deg>`: 각도 설정(`A0` ~ `A180`)
- `CMIN:<us>`: 0도 펄스폭 설정(`500`~`3000`)
- `CMAX:<us>`: 180도 펄스폭 설정(`500`~`3000`)
- `STATUS`: 현재 각도/펄스폭 상태 출력
- `HELP`: 도움말 출력

서보 초기값:
- 초기 각도: `90 deg`
- 기본 캘리브레이션: `CMIN=500us`, `CMAX=2500us`

## 4) ADC 샘플링 구성

### 4.1 ADC 설정
- ADC1, 12-bit, Scan mode enabled
- External trigger: TIM3 TRGO (rising edge)
- Number of conversion: 3
- EOC selection: single conversion

채널 순서:
- Rank 1: ADC_CHANNEL_0 (PA0)
- Rank 2: ADC_CHANNEL_1 (PA1)
- Rank 3: ADC_CHANNEL_4 (PA4)

### 4.2 샘플링 주기(TIM3)
- Prescaler = 8399
- Period = 99
- TRGO = Update event

계산:
- TIM3 clock = 84MHz
- Counter clock = 84MHz / (8399 + 1) = 10kHz
- Update period = (99 + 1) / 10kHz = 10ms
- Sampling rate = 100Hz

## 6) Servo PWM 구성

### 6.1 TIM4 PWM
- TIM4 CH1 (PB6)
- Prescaler = 83
- Period = 19999
- PWM frequency = 50Hz(20ms)

계산:
- TIM4 clock = 84MHz
- Counter clock = 84MHz / (83 + 1) = 1MHz (1 tick = 1us)
- Period = 20000us = 20ms = 50Hz

### 6.2 각도 -> 펄스폭 변환
서보 펄스폭 계산식은 아래와 같음.

- `pulse_us = CMIN + ((angle * (CMAX - CMIN)) / 180)`

범위:
- angle: 0~180
- CMIN/CMAX: 500~3000us (단, CMIN < CMAX)

## 7) 런타임 동작 개요

### 7.1 공통 초기화
1. GPIO/UART/ADC/TIM3/TIM4 초기화
2. TIM4 PWM 시작
3. ADC 큐/서보 초기 상태 설정
4. TIM3 시작(ADC 트리거 타이머)
5. ADC 인터럽트 시작
6. UART 모드 선택

### 7.2 메인 루프
- Servo 모드: UART 명령 처리(`A`, `CMIN`, `CMAX`, `STATUS`, `HELP`)
- ADC 모드: UART 명령 처리(`START`, `STOP`, `STATUS`, `HELP`) + 큐 pop/CSV 출력

## 8) UART 출력 예시

### 8.1 ADC 모드
- `Input> STATUS`
- `[ADC STATUS]`
- `stream    : ON`
- `sample    : 100 Hz`

### 8.2 Servo 모드
- `Input> A90`
- `Angle set: 90 deg (1500us)`
- `Input> CMIN:600`
- `CMIN = 600 us (CMAX = 2500 us)`

## 9) CubeMX(.ioc) 반영 상태
현재 .ioc는 코드와 일치하는 설정 상태임.

- ADC External Trigger: `TIM3 TRGO`
- TIM3: `Prescaler=8399`, `Period=99`, `MasterOutputTrigger=TIM_TRGO_UPDATE`
- TIM4 PWM: `CH1`, `Prescaler=83`, `Period=19999`
- 핀 라벨 반영:
  - `PA0: ADC_CH0_IN0`
  - `PA1: ADC_CH1_IN1`
  - `PA4: ADC_CH2_IN4`
  - `PB6: SERVO_PWM_TIM4_CH1`

## 10) 핀맵 (STM32F411RE Nucleo-64)

| 기능 | MCU 핀 | 보드 핀명 | 주변장치/채널 | 비고 |
|---|---|---|---|---|
| ADC 입력 1 | PA0 | A0 | ADC1_IN0 (Rank 1) | 아날로그 입력 |
| ADC 입력 2 | PA1 | A1 | ADC1_IN1 (Rank 2) | 아날로그 입력 |
| ADC 입력 3 | PA4 | A2 | ADC1_IN4 (Rank 3) | 아날로그 입력 |
| 서보 PWM 출력 | PB6 | D10 | TIM4_CH1 (AF2) | SG-90 신호선 |
| UART TX | PA2 | D1 | USART2_TX | 로그/CSV 출력 |
| UART RX | PA3 | D0 | USART2_RX | 명령 입력 |
| 사용자 LED | PA5 | D13 | GPIO Output | LD2 |
| 사용자 버튼 | PC13 | B1 | GPIO Input/EXTI | 기본 버튼 |

내부 연결(외부 핀 없음):
- TIM3 Update Event(TRGO) -> ADC1 External Trigger

## 11) 빌드
- 명령: `cmake --build build/Debug -j4`
- 상태: 정상 빌드 확인

## 12) 주의 사항
- SG-90 전원은 가능하면 외부 5V 사용 권장
- 외부 전원 사용 시 STM32 보드와 GND 공통 연결 필수
- CSV 출력은 UART 트래픽을 크게 증가시키므로 필요 시 `STOP`으로 중지
- `printf` float 출력은 코드 크기 증가 가능

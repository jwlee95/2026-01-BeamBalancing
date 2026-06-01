# 2026-01-BeamBalancing 기능/설정 상세 문서

이 문서는 현재 `readme.md`와 동일한 기준으로, 코드 관점에서 구현 이유와 동작 흐름을 상세히 설명합니다.
대상 보드는 STM32F411RE Nucleo-64입니다.

---

## 1. 시스템 개요

이 프로젝트는 아래 기능을 동시에 수행합니다.

- TIM3 TRGO 기반 ADC 외부 트리거 샘플링 (100Hz)
- 3채널 ADC 프레임 큐잉
- TIM4 CH1 PWM 기반 SG-90 제어 (50Hz)
- UART 명령 인터페이스 기반 모드 선택 및 런타임 제어

핵심 설계 포인트는 타이머 분리입니다.

- TIM3: ADC 샘플링 타이밍 전용
- TIM4: 서보 PWM 출력 전용

이렇게 분리하면 샘플링 주기와 PWM 주기가 서로 간섭하지 않습니다.

---

## 2. 부팅 모드 선택

부팅 직후 UART에서 모드를 선택합니다.

- `1`: ADC 모드
- `2`: Servo 모드

출력 예:

- `=== Mode Select ===`
- `1: ADC Mode   - 3ch raw data CSV output`
- `2: Servo Mode - command interface (A, CMIN, CMAX, STATUS, HELP)`

선택 이후 메인 루프는 모드별 입력 파서를 사용합니다.

- ADC 모드: `ProcessAdcInput()`
- Servo 모드: `ProcessServoInput()`

---

## 3. ADC 경로 상세

### 3.1 ADC 트리거 경로

- ADC1 외부 트리거 소스: TIM3 TRGO
- 트리거 엣지: Rising
- Scan conversion: 3채널 

채널 순서:

- Rank1: ADC_CHANNEL_0 (PA0)
- Rank2: ADC_CHANNEL_1 (PA1)
- Rank3: ADC_CHANNEL_4 (PA4)

### 3.2 TIM3 설정과 샘플링 주기

- Prescaler = 8399
- Period = 99
- MasterOutputTrigger = TIM_TRGO_UPDATE

계산:

- TIM3 clock = 84MHz
- Counter clock = 84MHz / (8399 + 1) = 10kHz
- Update period = (99 + 1) / 10kHz = 10ms
- 샘플링 주파수 = 100Hz

### 3.3 인터럽트/큐 구조

ADC 인터럽트에서는 변환값을 프레임으로 누적하고 큐에 push만 수행합니다.
무거운 처리는 메인 루프에서 수행합니다.

- ISR: `HAL_ADC_ConvCpltCallback()` -> 프레임 완성 시 큐 push
- Main: 큐 pop -> CSV 출력

큐 정책:

- 길이: 256 프레임
- overflow 시 가장 오래된 프레임 폐기 후 최신 프레임 저장

---

## 4. ADC 모드 명령 인터페이스

ADC 모드 프롬프트: `Input>`

지원 명령:

- `START`: CSV 스트리밍 ON
- `STOP`: CSV 스트리밍 OFF
- `STATUS`: 현재 상태 출력
- `HELP`: 도움말 출력

상태 예시:

- stream ON/OFF
- sample rate
- 누적 프레임 수
- 큐 점유/오버플로우

참고:

- `STOP`은 ADC 자체를 끄는 것이 아니라 UART CSV 출력만 멈춥니다.
- 샘플링/큐 적재는 계속 진행됩니다.

---

## 5. Servo 경로 상세

### 5.1 TIM4 PWM

- 타이머: TIM4 CH1 (PB6)
- Prescaler = 83
- Period = 19999

계산:

- TIM4 clock = 84MHz
- Counter clock = 84MHz / (83 + 1) = 1MHz (1tick = 1us)
- PWM 주기 = 20000us = 20ms = 50Hz

### 5.2 각도 기반 제어

서보 제어는 퍼센트 방식이 아니라 각도 기반으로 동작합니다.

- 함수: `SetServoAngle(uint8_t angle)`
- 범위: `0~180 deg`

변환식:

- `pulse_us = CMIN + ((angle * (CMAX - CMIN)) / 180)`

기본값:

- 초기 각도: `90 deg`
- `CMIN = 500us`
- `CMAX = 2500us`

---

## 6. Servo 모드 명령 인터페이스

Servo 모드 프롬프트: `Input>`

지원 명령:

- `A<deg>`: 각도 설정 (`A0` ~ `A180`)
- `CMIN:<us>`: 0도 펄스폭 (`500~3000`)
- `CMAX:<us>`: 180도 펄스폭 (`500~3000`)
- `STATUS`: 현재 각도/캘리브레이션/현재 펄스폭 출력
- `HELP`: 도움말 출력

유효성 규칙:

- `CMIN < CMAX` 필수
- `CMAX > CMIN` 필수
- 범위 외 값 입력 시 에러 메시지 출력

---

## 7. UART 입력 처리 정책

ADC/Servo 모드 모두 동일한 기본 정책을 사용합니다.

- 1바이트 non-blocking 수신 (`HAL_UART_Receive(..., timeout=0)`)
- Enter(`\r`/`\n`) 시 한 줄 명령 확정
- 소문자 입력은 대문자로 변환 후 비교
- 처리 후 `Input>` 프롬프트 재출력

장점:

- 메인 루프 블로킹 없음
- 단순하고 유지보수 쉬운 파서 구조

---

## 8. CubeMX(.ioc) 동기화 상태

현재 `.ioc`와 코드 설정은 동기화되어 있습니다.

- ADC external trigger: `TIM3 TRGO`
- TIM3: `Prescaler=8399`, `Period=99`, `MasterOutputTrigger=TIM_TRGO_UPDATE`
- TIM4 PWM: `CH1`, `Prescaler=83`, `Period=19999`

핀 라벨:

- `PA0: ADC_CH0_IN0`
- `PA1: ADC_CH1_IN1`
- `PA4: ADC_CH2_IN4`
- `PB6: SERVO_PWM_TIM4_CH1`

---

## 9. 핀맵 (STM32F411RE Nucleo-64)

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

---

## 11. 빌드/운영 메모

빌드:

- `cmake --build build/Debug -j4`

운영 시 주의:

- SG-90 전원은 외부 5V 권장
- 외부 5V 사용 시 Nucleo GND와 공통 접지 필수
- CSV 스트리밍은 UART 대역폭을 크게 사용하므로 필요 시 `STOP` 사용
- `%f` 출력 사용 시 코드 크기 증가 가능


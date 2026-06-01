# 회로도 툴용 부품/네트 정리

이 문서는 KiCad, EasyEDA, Altium에서 수동 배선할 때 사용할 기준입니다.

## 부품 ID 권장
- U1: STM32 Nucleo-F411RE
- J1: Optical distance sensor
- J2: SG-90 servo
- J3: HC-SR04
- U2: ECHO 레벨시프터(또는 저항 분배기)
- PWR1: External +5V
- J6: UART Debug Header (옵션)

## HC-SR04 ECHO 레벨 변환
- 권장 분압기: R1=1k (상단), R2=2k (하단)
- 연결:
  - J3.ECHO -> R1 -> 노드 ECHO_DIV
  - ECHO_DIV -> R2 -> GND
  - ECHO_DIV -> U1.PA8/D7 (TIM1_CH1)

## 전원 규칙
- SG-90은 외부 +5V 권장
- U1 GND, J1 GND, J2 GND, J3 GND, PWR1 GND는 반드시 공통
- 광학 센서 전원은 센서 데이터시트 기준(3.3V/5V) 확인

## 네트 이름 권장
- ADC0_OPT
- SERVO_PWM
- HCSR04_TRIG
- HCSR04_ECHO_3V3
- +3V3
- +5V_EXT
- GND

# STM32_clock
ARM core인 STM32f4를 사용해서 알람 시계 프로젝트를 만들어보았습니다. 

 융합기술교육원에서 프로젝트에서 STM32 알람시계를 만들어 보았습니다.(유튜브 동영상)

 [![Grepp_A#](http://img.youtube.com/vi/ASdZDDtxkZc/0.jpg)]( https://youtu.be/ASdZDDtxkZc) 

https://youtu.be/ASdZDDtxkZc

## 1. 프로젝트 목표
- STM32F4보드에서 임베디드 각 기능들을 이용하여 알람시계를 구현한다.

![image](https://user-images.githubusercontent.com/81784631/135090650-4fdbfe6f-7d0c-40af-844c-9b4d829da39e.png)


## 2. 프로젝트 상세 내용
HW: stm32F432zi보드, LCM1602쉴드  
SW: 사용하여 모드(시계설정, 알람 시계, 알람노래) 구현한다. Hal라이브러리로 GPIO, ADC, I2C,UART, EEPROME

- 알람 셋팅 모드
    만약 얄람이 울리면 화면이 반짝거린다.
- 현재 시간 모드
    ADC 조이스틱으로 숫자 컨트롤
- 알람 음악 설정 모드 (5개 음악)
  
- 
![image](https://user-images.githubusercontent.com/81784631/135090735-d21209f8-27ab-4538-b80f-efc94f07edd8.png)
